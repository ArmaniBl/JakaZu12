"""Informational last cloud update; never a motion-readiness condition."""
import json
import math
from pathlib import Path
import time


def latest_update(workspace):
    root=Path(workspace)/'scans'
    try:
        value=json.loads((root/'scan_status.json').read_text())['updated_unix']
    except (OSError,ValueError,KeyError,TypeError):
        try:
            import numpy as np
            with np.load(root/'preview-cloud-cache.npz') as cache:value=float(cache['updated_unix'])
        except (OSError,ValueError,KeyError,TypeError):return None
    if isinstance(value,bool) or not isinstance(value,(int,float)) or not math.isfinite(value) or not 0<value<=time.time()+30:
        return None
    return float(value)
