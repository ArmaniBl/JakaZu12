"""Share the lifetime of a real welding operation with the local scanner."""
from contextlib import contextmanager
import fcntl
import os
from pathlib import Path
import tempfile


def _path():
    domain = int(os.environ.get('ROS_DOMAIN_ID', '0'))
    return Path(tempfile.gettempdir()) / f'jaka-welding-operation-{os.getuid()}-{domain}.lock'


@contextmanager
def welding_operation():
    with _path().open('a') as lock:
        try:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            raise RuntimeError('Another real welding operation is already running') from None
        yield


def welding_operation_active():
    with _path().open('a') as lock:
        try:
            fcntl.flock(lock, fcntl.LOCK_SH | fcntl.LOCK_NB)
        except BlockingIOError:
            return True
        return False
