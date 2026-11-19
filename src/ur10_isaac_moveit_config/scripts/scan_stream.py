"""Sequential single-view acquisition controller; no robot or platform motion."""
import json
from pathlib import Path
import subprocess
from urllib.request import Request, urlopen


def scanner_request(url, action):
    routes = {'state': 'state', 'capture': 'capture'}
    route = routes[action]
    if url == 'windows-local':
        root = next(p for p in Path(__file__).resolve().parents
                    if (p / 'integrations/read_scan.ps1').is_file())
        script = subprocess.check_output(['wslpath', '-w', str(root / 'integrations/read_scan.ps1')], text=True).strip()
        result = subprocess.run(['powershell.exe', '-NoProfile', '-NonInteractive', '-ExecutionPolicy',
                                 'Bypass', '-File', script, '-Action', action],
                                capture_output=True, timeout=20)
        if result.returncode:
            raise ValueError(result.stderr.decode('utf-8', errors='replace').strip()[-2000:])
        return json.loads(result.stdout)
    if not url.startswith(('http://', 'https://')):
        raise ValueError('Scanner URL must use HTTP(S)')
    data = json.dumps(dict(require_setup=True, turntable=False, positions=1)).encode() if action == 'capture' else None
    request = Request(url.rstrip('/') + '/api/spectrum/' + route, data=data,
                      headers={'Content-Type': 'application/json'})
    with urlopen(request, timeout=15) as response:
        return json.load(response)


class CaptureLoop:
    def __init__(self, request, continuous=False):
        self.request = request
        self.continuous = continuous
        self.owned_capture = None
        self.paused = None
        self.failures = 0
        self.last_terminal = None
        self.state = 'WAITING'
        self.displayed_capture = None
        self.retired_captures = set()

    def poll(self, fetch):
        state = self.request('state')
        job = state.get('job') or {}
        self.state = job.get('state', 'WAITING')
        if self.state in ('CAPTURING', 'DECODING', 'PLANNING', 'COMPARING'):
            return None
        terminal = (job.get('capture_id'), self.state)
        if terminal != self.last_terminal and job.get('capture_id') == self.owned_capture and self.owned_capture:
            self.last_terminal = terminal
            if self.state == 'CANCELLED':
                self.paused = 'Capture cancelled by operator; restart preview to resume acquisition'
            elif self.state == 'FAILED':
                self.failures += 1
                if self.failures >= 3:
                    self.paused = 'Three consecutive failed captures: ' + str(job.get('reason', ''))
            elif state.get('fresh_capture_for_task'):
                self.failures = 0
        bundle = fetch() if state.get('fresh_capture_for_task') else None
        if isinstance(bundle,dict):
            capture=bundle.get('capture_id')
            active=state.get('active_task') or {}
            expected=active.get('capture_id')
            # The state and export are separate HTTP reads. A task switch or
            # delayed response must never roll the displayed scene backwards.
            if (capture in self.retired_captures or not capture
                    or (expected and capture!=expected)
                    or (active.get('id') and bundle.get('task_id')!=active['id'])):
                raise ValueError('Outdated scanner export rejected; waiting for current capture')
            if self.displayed_capture and capture!=self.displayed_capture:
                self.retired_captures.add(self.displayed_capture)
            self.displayed_capture=capture
        if self.continuous and not self.paused and state.get('active_task'):
            try:
                result = self.request('capture')
                self.owned_capture = result['job']['capture_id']
                self.state = result['job']['state']
            except Exception as error:
                self.paused = 'Acquisition could not start; restart preview after resolving: ' + str(error)
        return bundle
