"""Read the JAKA status stream without writing to the controller socket."""
import codecs
import json
import math
import socket
import threading
import time


def packets(connection):
    decoder = json.JSONDecoder()
    utf8 = codecs.getincrementaldecoder('utf-8')(errors='replace')
    buffer = ''
    while True:
        chunk = connection.recv(16384)
        if not chunk:
            return
        buffer += utf8.decode(chunk)
        buffer = buffer.lstrip('\x00 \r\n\t')
        while buffer:
            try:
                value, end = decoder.raw_decode(buffer)
            except json.JSONDecodeError:
                if len(buffer) > 65536:
                    raise ValueError('Invalid or oversized JAKA status packet')
                break
            if isinstance(value, dict):
                yield value
            buffer = buffer[end:].lstrip('\x00 \r\n\t')


def _six(values):
    if (isinstance(values, list) and len(values) == 6
            and all(type(v) in (int, float) and math.isfinite(v) for v in values)):
        return values
    return None


class FeedbackCapture:
    def __init__(self, ip):
        self.report = dict(source='JAKA read-only TCP port 10000', stages=[], connection_errors=[])
        self.lock = threading.Lock()
        self.stop = threading.Event()
        self.stage = None
        self.worker = threading.Thread(target=self._read, args=(ip,), daemon=True)
        self.worker.start()

    def begin(self, name, trajectory):
        point = trajectory.points[-1]
        duration = point.time_from_start.sec + point.time_from_start.nanosec / 1e9
        with self.lock:
            self.stage = dict(name=name, started_unix=time.time(), planned_duration_sec=duration,
                              target_joints_rad=dict(zip(trajectory.joint_names, point.positions)), samples=[])
            self.report['stages'].append(self.stage)
            self.last_sample = 0.
            self.last_fault = None

    def end(self, outcome):
        with self.lock:
            if self.stage is not None:
                self.stage.update(finished_unix=time.time(), **outcome)
                self.stage = None

    def close(self):
        self.end(dict(status='interrupted'))
        self.stop.set()
        self.worker.join(1.6)

    def record(self, packet):
        now = time.monotonic()
        with self.lock:
            if self.stage is None:
                return
            fault = tuple(packet.get(key) for key in ('errcode', 'protective_stop', 'emergency_stop', 'powered_on'))
            if now - self.last_sample < .1 and fault == self.last_fault:
                return
            self.last_sample, self.last_fault = now, fault
            sample = dict(unix=time.time())
            for key in ('errcode', 'errmsg', 'protective_stop', 'emergency_stop', 'powered_on', 'inpos'):
                if key in packet and isinstance(packet[key], (str, int, bool)):
                    sample[key] = packet[key]
            actual = _six(packet.get('joint_actual_position'))
            commanded = _six(packet.get('joint_position'))
            if actual is not None:
                sample['actual_joint_position'] = actual
            if commanded is not None:
                sample['commanded_joint_position'] = commanded
            if actual is not None and commanded is not None:
                error = max(abs(a - b) for a, b in zip(actual, commanded))
                sample['max_joint_error_controller_units'] = error
                old = self.stage.get('max_joint_error_controller_units', 0.)
                self.stage['max_joint_error_controller_units'] = max(old, error)
            self.stage['samples'].append(sample)

    def _read(self, ip):
        while not self.stop.is_set():
            try:
                with socket.create_connection((ip, 10000), timeout=1) as connection:
                    if self.stop.is_set():
                        return
                    connection.settimeout(1)
                    for packet in packets(connection):
                        if self.stop.is_set():
                            return
                        self.record(packet)
                    raise OSError('JAKA status stream closed')
            except (OSError, ValueError) as error:
                if self.stop.is_set():
                    return
                with self.lock:
                    errors = self.report['connection_errors']
                    if len(errors) < 20:
                        errors.append(dict(unix=time.time(), error=str(error)))
                self.stop.wait(1)
