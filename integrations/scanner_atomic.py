"""Bounded retry when a Windows projector reader briefly locks its command file."""
import time
import json
import math


def capture_exposure(requested, settings_path):
    """Read one exposure for the whole sequence, including resumed captures."""
    value = requested
    if settings_path.is_file():
        value = json.loads(settings_path.read_text(encoding='utf-8')).get('exposure_us', value)
    if isinstance(value, bool):
        raise ValueError('Exposure must be a number in microseconds')
    value = float(value)
    if not math.isfinite(value) or not 100 <= value <= 100000:
        raise ValueError('Exposure must be between 100 and 100000 microseconds')
    return value


def read_text_when_available(path,timeout=2.):
    deadline=time.monotonic()+timeout
    while True:
        try:return path.read_text()
        except FileNotFoundError:return ''
        except PermissionError:
            if time.monotonic()>=deadline:raise
            time.sleep(.02)


def select_projector(displays,display_id):
    candidates=[d for d in displays if not d['builtin'] and not d['main'] and not d['mirrors']]
    if display_id==0:
        if len(candidates)!=1:
            raise ValueError('Expected one independent external projector; connect HDMI in extended mode or set SPECTRUM_PROJECTOR_DISPLAY')
        return candidates[0]
    display=next((d for d in candidates if d['id']==display_id),None)
    if display is None:raise ValueError('Configured projector is unavailable; check HDMI and extended display mode')
    return display


def replace_when_available(source, target, timeout=2.):
    deadline = time.monotonic()+timeout
    while True:
        try:
            return source.replace(target)
        except PermissionError:
            if time.monotonic()>=deadline:
                raise
            time.sleep(.02)
