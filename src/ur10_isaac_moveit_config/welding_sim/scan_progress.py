"""Read-only scanner progress. Never participates in welding readiness."""
from datetime import datetime
import json
from pathlib import Path
import re
import subprocess
import time


def progress_payload(state, now=None):
    now = time.time() if now is None else now
    job = state.get('job') or {}
    phase = job.get('state', 'WAITING')
    capture_id = job.get('capture_id')
    result = dict(state=phase, capture_id=capture_id, observed_unix=now,
                  completed_frames=None, total_frames=None, percent=None,
                  started_unix=None, finished_unix=None)
    if phase == 'CAPTURING':
        match = re.fullmatch(r'Pattern (\d+)/(\d+) captured', str(job.get('progress', '')))
        if match:
            current, total = map(int, match.groups())
            if 0 <= current <= total <= 10000 and total > 0:
                result.update(completed_frames=current, total_frames=total,
                              percent=round(current*100/total))
    captures = (state.get('active_task') or {}).get('captures') or []
    attempt = next((c for c in reversed(captures) if c.get('id') == capture_id), {})
    for source, target in [('started_at', 'started_unix'), ('completed_at', 'finished_unix')]:
        try:
            value = datetime.fromisoformat(attempt[source].replace('Z', '+00:00'))
            if value.tzinfo is not None and 0 < value.timestamp() <= now+30:
                result[target] = value.timestamp()
        except (KeyError, TypeError, ValueError):
            pass
    return result


def scanner_state(workspace):
    script = Path(workspace)/'integrations/read_scan.ps1'
    windows_path = subprocess.check_output(['wslpath', '-w', str(script)], text=True).strip()
    result = subprocess.run(['powershell.exe', '-NoProfile', '-NonInteractive',
        '-ExecutionPolicy', 'Bypass', '-File', windows_path, '-Action', 'state'],
        capture_output=True, timeout=20)
    if result.returncode:
        raise OSError('Scanner state unavailable')
    return json.loads(result.stdout)


def monitor_progress(controller, stop):
    while not stop.is_set():
        try:
            progress = progress_payload(scanner_state(controller.workspace))
        except (OSError, ValueError, TypeError, subprocess.SubprocessError):
            progress = dict(state='UNAVAILABLE', observed_unix=time.time())
        with controller.lock:
            controller.scan_progress = progress
        stop.wait(3.)
