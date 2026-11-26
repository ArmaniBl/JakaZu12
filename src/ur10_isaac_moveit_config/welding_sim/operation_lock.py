"""Share the lifetime of a real welding operation with the local scanner."""
from contextlib import contextmanager
import fcntl
import os
from pathlib import Path
import tempfile


def _path(scope="jaka"):
    domain = int(os.environ.get('ROS_DOMAIN_ID', '0'))
    return Path(tempfile.gettempdir()) / f'{scope}-welding-operation-{os.getuid()}-{domain}.lock'


@contextmanager
def welding_operation(scope="jaka"):
    with _path(scope).open('a') as lock:
        try:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            label = 'real welding' if scope == "jaka" else 'Isaac replay'
            raise RuntimeError(f'Another {label} operation is already running') from None
        yield


def welding_operation_active():
    with _path().open('a') as lock:
        try:
            fcntl.flock(lock, fcntl.LOCK_SH | fcntl.LOCK_NB)
        except BlockingIOError:
            return True
        return False
