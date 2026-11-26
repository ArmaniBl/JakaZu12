"""Sequential single-view acquisition controller; no robot or platform motion."""
import json
import re
from pathlib import Path
import subprocess
from urllib.request import Request, urlopen


def scanner_request(url, action, task=None):
    routes = {'state': 'state', 'capture': 'capture', 'cancel': 'cancel', 'new-task': 'tasks'}
    route = routes[action]
    if url == 'windows-local':
        root = next(p for p in Path(__file__).resolve().parents
                    if (p / 'integrations/read_scan.ps1').is_file())
        script = subprocess.check_output(['wslpath', '-w', str(root / 'integrations/read_scan.ps1')], text=True).strip()
        command = ['powershell.exe', '-NoProfile', '-NonInteractive', '-ExecutionPolicy',
                   'Bypass', '-File', script, '-Action', action]
        if action == 'new-task':
            command += ['-TaskName', task['name'], '-PartId', task['part_id']]
        result = subprocess.run(command, capture_output=True, timeout=20)
        if result.returncode:
            raise ValueError(result.stderr.decode('utf-8', errors='replace').strip()[-2000:])
        return json.loads(result.stdout)
    if not url.startswith(('http://', 'https://')):
        raise ValueError('Scanner URL must use HTTP(S)')
    data = json.dumps(dict(require_setup=True, turntable=False, positions=1)).encode() if action == 'capture' else None
    if action == 'new-task':
        data = json.dumps(task).encode()
    if action == 'cancel':
        data = b'{}'
    request = Request(url.rstrip('/') + '/api/spectrum/' + route, data=data,
                      headers={'Content-Type': 'application/json'})
    with urlopen(request, timeout=15) as response:
        return json.load(response)


def workpiece_key(task_name):
    match = re.fullmatch(r'MoveIt-(.+)-[0-9a-f]{32}', task_name or '')
    return match[1] if match else None


class CaptureLoop:
    def __init__(self, request, continuous=False):
        self.request = request
        self.continuous = continuous
        self.owned_capture = None
        self.paused = None
        self.failures = 0
        self.last_terminal = None
        self.state = 'WAITING'
        self.last_failure = None
        self.displayed_capture = None
        self.early_capture = None
        self.retired_captures = set()
        self.applied_restart = None
        self.motion_cancel = None
        self.motion_discards = set()

    def pause_for_motion(self, job):
        self.state = 'PAUSED_MOTION'
        if not self.owned_capture or job.get('capture_id') != self.owned_capture:
            return
        # Discard a capture that overlaps the operation even if it finishes
        # between the status read and the cancellation request.
        self.motion_discards.add(self.owned_capture)
        if job.get('state') not in ('CAPTURING', 'DECODING', 'PLANNING'):
            self.owned_capture = None
            self.motion_cancel = None
        elif self.motion_cancel != self.owned_capture:
            self.request('cancel')
            self.motion_cancel = self.owned_capture

    def poll(self, fetch, restart_name=None, *, moving=lambda: False, reconstruct=None):
        if self.continuous and moving() and self.owned_capture is None:
            self.state = 'PAUSED_MOTION'
            return None
        state = self.request('state')
        job = state.get('job') or {}
        if self.continuous and moving():
            self.pause_for_motion(job)
            return None
        if self.motion_cancel is not None and job.get('state') not in ('CAPTURING', 'DECODING', 'PLANNING'):
            if self.owned_capture == self.motion_cancel:
                self.owned_capture = None
            self.motion_cancel = None
        self.state = job.get('state', 'WAITING')
        if self.state == 'FAILED' and not reconstruct:
            self.last_failure = job.get('reason') or job.get('progress')
        active = state.get('active_task') or {}
        requested_workpiece = workpiece_key(restart_name)
        if (self.continuous and requested_workpiece
                and workpiece_key(active.get('name')) == requested_workpiece
                and not self.paused and active.get('capture_count', 0) < 200):
            # Reapplying the same CAD in RViz can happen while the physical scan
            # is finishing. Keep this task so its new cloud can be exported.
            self.applied_restart = restart_name
            restart_name = None
        if self.state in ('CAPTURING', 'DECODING', 'PLANNING', 'COMPARING'):
            capture = job.get('capture_id')
            if (not reconstruct or self.state != 'DECODING' or restart_name
                    or not capture or capture != self.owned_capture
                    or capture == self.early_capture or capture in self.motion_discards):
                return None
            self.early_capture = capture
            # The Windows legacy reconstruction runs after the raw archive is
            # complete. Process the same archive while that stage is running.
            try:
                bundle = reconstruct(capture, active.get('id'))
            except (OSError, ValueError, KeyError, subprocess.SubprocessError):
                return None  # Retry on the terminal job state.
            latest = self.request('state')
            latest_job = latest.get('job') or {}
            if self.continuous and moving():
                self.pause_for_motion(latest_job)
                return None
            if (latest_job.get('capture_id') != capture
                    or latest_job.get('state') not in ('DECODING', 'COMPLETE', 'FAILED')
                    or (latest.get('active_task') or {}).get('id') != active.get('id')):
                return None
            self.accept_bundle(bundle, capture, active.get('id'))
            self.failures = 0
            self.last_failure = None
            return bundle
        if self.continuous and restart_name:
            if not active.get('part_id'):
                raise ValueError('Select a workpiece in the scanner application before restarting acquisition')
            if active.get('name') != restart_name:
                self.request('new-task', dict(name=restart_name, part_id=active['part_id']))
            # Do not fetch a result from the previous workpiece in this poll.
            self.paused = self.owned_capture = self.last_terminal = None
            self.failures = 0
            self.last_failure = None
            if moving():
                self.state = 'PAUSED_MOTION'
                return None
            result = self.request('capture')
            self.owned_capture = result['job']['capture_id']
            self.state = result['job']['state']
            self.applied_restart = restart_name
            return None
        terminal = (job.get('capture_id'), self.state)
        bundle = None
        if terminal != self.last_terminal and job.get('capture_id') == self.owned_capture and self.owned_capture:
            self.last_terminal = terminal
            already_displayed = reconstruct and self.owned_capture == self.displayed_capture
            if (reconstruct and not already_displayed and self.state in ('COMPLETE', 'FAILED')
                    and self.owned_capture not in self.motion_discards):
                try:
                    bundle = reconstruct(self.owned_capture, active.get('id'))
                except Exception as error:
                    self.last_failure = str(error)
            if self.continuous and moving():
                self.pause_for_motion(job)
                return None
            if bundle is not None or already_displayed:
                self.failures = 0
                self.last_failure = None
            elif self.state == 'CANCELLED' and self.owned_capture not in self.motion_discards:
                self.paused = 'Capture cancelled by operator; restart preview to resume acquisition'
            elif (self.state == 'FAILED' or (reconstruct and self.state == 'COMPLETE')) and self.owned_capture not in self.motion_discards:
                self.last_failure = self.last_failure or job.get('reason') or job.get('progress')
                self.failures += 1
                if self.failures >= 3:
                    self.paused = 'Three consecutive failed captures: ' + str(self.last_failure or job.get('reason', ''))
            elif state.get('fresh_capture_for_task'):
                self.failures = 0
                self.last_failure = None
        if self.continuous and moving():
            self.pause_for_motion(job)
            return None
        if not reconstruct and state.get('fresh_capture_for_task') and active.get('capture_id') not in self.motion_discards:
            bundle = fetch()
        if self.continuous and moving():
            self.pause_for_motion(job)
            return None
        if isinstance(bundle, dict) and bundle.get('capture_id') in self.motion_discards:
            bundle = None
        if isinstance(bundle, dict):
            expected = job.get('capture_id') if reconstruct else active.get('capture_id')
            self.accept_bundle(bundle, expected, active.get('id'))
        if self.continuous and not self.paused and state.get('active_task'):
            try:
                result = self.request('capture')
                self.owned_capture = result['job']['capture_id']
                self.state = result['job']['state']
            except Exception as error:
                self.paused = 'Acquisition could not start; restart preview after resolving: ' + str(error)
        return bundle


    def accept_bundle(self, bundle, expected_capture, expected_task):
        capture = bundle.get('capture_id')
        if (capture in self.retired_captures or not capture
                or (expected_capture and capture != expected_capture)
                or (expected_task and bundle.get('task_id') != expected_task)):
            raise ValueError('Outdated scanner export rejected; waiting for current capture')
        if self.displayed_capture and capture != self.displayed_capture:
            self.retired_captures.add(self.displayed_capture)
        self.displayed_capture = capture
