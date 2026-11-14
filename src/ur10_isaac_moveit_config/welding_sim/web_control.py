"""Local browser control of the existing welding CLI and owned ROS launch."""
from collections import deque
from dataclasses import dataclass, asdict
from datetime import datetime, timezone
import json
import math
import os
from pathlib import Path
import re
import signal
import subprocess
import sys
import threading
import time
import uuid

from .geometry import FACES, box_edges, face_contour, select_edges
from .geometry import WorkpieceGeometry
from .profiles import ISAAC
from .orientation import tilted_orientation
from ament_index_python.packages import get_package_share_directory


@dataclass(frozen=True)
class Settings:
    object: str = "welding_workpiece"
    selection: str = "face"
    face: str = "z_plus"
    edge: str = "x_plus__z_plus"
    begin: float = 0.0
    end: float = 1.0
    offset: float = 0.03
    clearance: float = 0.05
    step: float = 0.005
    speed: float = 0.02
    scaling: float = 0.05
    roll: float = 0.0
    orientation_mode: str = 'auto'
    tilt: float = 0.0
    tilt_direction: float = 0.0
    max_tilt: float = 45.0
    orientation_timeout: float = 120.0

    @classmethod
    def parse(cls, data):
        if not isinstance(data, dict):
            raise ValueError("Настройки должны быть объектом JSON")
        defaults = asdict(cls())
        unknown = set(data)-set(defaults)
        if unknown:
            raise ValueError("Неизвестные настройки: " + ", ".join(sorted(unknown)))
        defaults.update(data)
        for name in ("begin", "end", "offset", "clearance", "step", "speed", "scaling", "roll",
                     'tilt','tilt_direction','max_tilt','orientation_timeout'):
            if isinstance(defaults[name], bool):
                raise ValueError(f"Некорректное число: {name}")
            try:
                defaults[name] = float(defaults[name])
            except (ValueError, TypeError):
                raise ValueError(f"Некорректное число: {name}") from None
            if not math.isfinite(defaults[name]):
                raise ValueError(f"Некорректное число: {name}")
        result = cls(**defaults)
        if (result.orientation_mode not in ('auto','fixed') or not 0<=result.tilt<=result.max_tilt<=60
                or not 5<=result.orientation_timeout<=300):
            raise ValueError('Наклон: 0 ≤ заданный ≤ максимальный ≤ 60°; время подбора: 5–300 с')
        if not isinstance(result.object, str) or not 1 <= len(result.object) <= 128:
            raise ValueError("Укажите ID объекта")
        if result.object == 'floor':
            raise ValueError('floor is reserved for the collision floor')
        if result.selection not in ("face", "edge"):
            raise ValueError("Выберите грань или ребро")
        for name in (result.face, result.edge):
            if not isinstance(name, str) or not re.fullmatch(r'[A-Za-z0-9_:.-]{1,128}', name):
                raise ValueError('Некорректный ID грани или ребра')
        if not 0 <= result.begin < result.end <= 1:
            raise ValueError("Начало участка должно быть меньше конца, в пределах 0–100%")
        for name, low, high in (("offset", 0, .5), ("clearance", 0, .5),
                                ("speed", 0, .1), ("scaling", 0, 1.0)):
            if not low < getattr(result, name) <= high:
                raise ValueError(f"Параметр {name} должен быть больше {low} и не больше {high}")
        if not .001 <= result.step <= .02:
            raise ValueError("Шаг должен быть от 1 до 20 мм")
        return result

    def argv(self, executable, mode, output):
        if mode not in ("preview", "plan", "execute"):
            raise ValueError("Неизвестный режим")
        args = [sys.executable, str(executable), "--mode", mode, "--object", self.object,
                "--output", str(output)]
        args += ["--face", self.face] if self.selection == "face" else ["--edge", self.edge]
        for name in ("begin", "end", "offset", "clearance", "step", "speed", "scaling", "roll",
                     'orientation_mode','tilt','tilt_direction','max_tilt','orientation_timeout'):
            args.extend(["--"+name.replace('_','-'), str(getattr(self, name))])
        return args


def geometry_payload(edges, settings, demo=False):
    geometry = edges if isinstance(edges, WorkpieceGeometry) else WorkpieceGeometry(
        'box', edges, {f: face_contour(edges, f) for f in FACES})
    edges = geometry.edges
    faces = list(geometry.contours.values())
    face = settings.face if settings.face in geometry.contours else next(iter(geometry.contours))
    edge = settings.edge if settings.edge in {e.id for e in edges} else edges[0].id
    selected = geometry.contour(face) if settings.selection == 'face' else geometry.select([edge])[0]
    seam, tool, orientation = selected.path(settings.begin, settings.end, settings.offset,
                                           settings.step, math.radians(settings.roll))
    orientation=tilted_orientation(orientation,settings.tilt,settings.tilt_direction)
    return {"demo": demo, "shape": geometry.shape, 'selected_face': face, 'selected_edge': edge,
            "faces": [{"id": c.id.split(":", 1)[1], "vertices": c.vertices[:-1].tolist(),
                                     "normal": c.outward.tolist(), "length": c.length} for c in faces],
            "edges": [{"id": e.id, "faces": e.faces, "start": e.start.tolist(),
                       "end": e.end.tolist(), "length": e.length,
                       'vertices': (e.vertices.tolist() if hasattr(e, 'vertices') else [e.start.tolist(), e.end.tolist()])} for e in edges],
            "seam": seam.tolist(), "tool": tool.tolist(), "orientation": orientation.tolist(),
            "normal": selected.outward.tolist(),
            "length": selected.length*(settings.end-settings.begin)}


class Controller:
    """Own child processes; never run shell text supplied by the browser."""
    def __init__(self, workspace, executable, profile=ISAAC, robot_config=None, return_pose=None):
        self.workspace = Path(workspace)
        self.executable = Path(executable)
        self.profile, self.robot_config, self.return_pose = profile, robot_config, return_pose
        self.lock = threading.RLock()
        self.commands = threading.Lock()
        self.logs = deque(maxlen=1200)
        self.sequence = 0
        self.jobs = []
        self.job_process = None
        self.scene_process = None
        self.job = None
        self.scene_stopping = False
        self.telemetry = {"isaac": False, "feedback": False, "moveit": False,
                          "controller": False, "objects": [], "error": None,
                          "robot": {"available": False, "frame": "world", "points": [],
                                    "reason": "Ожидаем состояние робота"}}
        self.object_edges = {}
        self.last_telemetry = 0.0
        self.report_root = self.workspace / "diagnostics" / "web"
        self.report_root.mkdir(parents=True, exist_ok=True)

    def log(self, source, text):
        with self.lock:
            self.sequence += 1
            self.logs.append({"seq": self.sequence, "source": source,
                              "time": datetime.now().astimezone().strftime("%H:%M:%S"), "text": text})

    def status(self, since=0):
        with self.lock:
            health = dict(self.telemetry)
            if time.monotonic()-self.last_telemetry > 4:
                health.update(isaac=False, feedback=False, moveit=False, controller=False, objects=[])
                health["robot"] = {"available": False, "frame": "world", "points": [],
                                   "reason": "Нет актуальной связи с ROS"}
            return {"health": health, 'robot_profile': self.profile.payload(), "job": dict(self.job) if self.job else None,
                    "history": [dict(job) for job in reversed(self.jobs[-12:])],
                    "scene": {"owned": bool(self.scene_process and self.scene_process.poll() is None),
                              "stopping": self.scene_stopping},
                    "logs": [entry for entry in self.logs if entry["seq"] > since], "sequence": self.sequence}

    def geometry(self, settings):
        with self.lock:
            live = self.object_edges.get(settings.object)
            edges = live or box_edges([.3, .3, .2], [-1, 0, .1])
        return geometry_payload(edges, settings, demo=live is None)

    def _read(self, process, source, job=None):
        for line in process.stdout:
            text = re.sub(r"\x1b\[[0-9;]*m", "", line.rstrip())
            self.log(source, text)
            if job:
                with self.lock:
                    if 'Orientation search:' in text:
                        job['phase']='Подбор наклона и поворота горелки'
                    elif "Planning " in text:
                        job["phase"] = "Планирование маршрута"
                    elif "Preview published" in text:
                        job["phase"] = "Предпросмотр в RViz"
                    elif "Executing " in text:
                        job["phase"] = ("Возврат в сохранённую позу" if "workpiece:return" in text else
                                        "Подвод к заготовке" if ":approach" in text else "Проход по контуру и отвод")
        code = process.wait()
        process.stdout.close()
        with self.lock:
            if job:
                reports = sorted((self.report_root / job["id"]).glob("*.json"))
                report = None
                if reports:
                    try:
                        report = json.loads(reports[-1].read_text())
                    except (ValueError, OSError):
                        pass
                job["exit_code"] = code
                job["finished"] = time.time()
                job["report"] = bool(report)
                job["result"] = report.get("status") if report else None
                job["error"] = report.get("error") if report else None
                job['chosen_orientation'] = report.get('chosen_orientation') if report else None
                job["state"] = ("cancelled" if job.get("stopping") and (not report or report.get("status") == "interrupted")
                                else "succeeded" if code == 0 and report and report.get("status") in ("preview", "planned", "executed")
                                else "failed")
                job["phase"] = {"cancelled": "Остановлено", "succeeded": "Готово", "failed": "Ошибка процесса"}[job["state"]]
            else:
                self.scene_stopping = False
        self.log(source, f"Процесс завершён, код {code}")

    def start(self, mode, settings):
        if mode not in ("preview", "plan", "execute"):
            raise ValueError("Неизвестный режим")
        with self.commands:
            with self.lock:
                current = self.job_process
                if current and current.poll() is None:
                    if self.job["mode"] != "preview":
                        raise ValueError("Уже выполняется операция. Дождитесь завершения или нажмите «Стоп»")
                    self._stop_job()
            if current and current.poll() is None:
                try:
                    current.wait(timeout=8)
                except subprocess.TimeoutExpired:
                    raise ValueError("Предпросмотр ещё останавливается") from None
            with self.lock:
                health = self.status()["health"]
                if not health["moveit"] or settings.object not in self.object_edges:
                    raise ValueError("Сначала запустите сцену и дождитесь появления заготовки")
                geometry = self.object_edges[settings.object]
                if isinstance(geometry, WorkpieceGeometry):
                    geometry.contour(settings.face) if settings.selection == 'face' else geometry.select([settings.edge])
                if mode != "preview" and not (health["feedback"] and (self.profile.real or health['isaac'])):
                    raise ValueError("Нет свежей обратной связи выбранного робота")
                if mode == "execute" and not health["controller"]:
                    raise ValueError("Action контроллера выбранного робота недоступен")
                job_id = uuid.uuid4().hex
                output = self.report_root / job_id
                job = {"id": job_id, "mode": mode, "settings": asdict(settings), "state": "running",
                       "phase": "Запуск", "started": time.time(), "stopping": False, "report": False}
                environment = dict(os.environ, PYTHONUNBUFFERED="1", RCUTILS_COLORIZED_OUTPUT="0")
                argv = settings.argv(self.executable, mode, output)+['--robot', self.profile.name]
                if self.robot_config:
                    argv += ['--robot-config', str(self.robot_config)]
                if self.return_pose:
                    argv += ['--return-pose', str(self.return_pose)]
                process = subprocess.Popen(argv, cwd=self.workspace,
                                           env=environment, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                           text=True, errors="replace", start_new_session=True)
                self.job, self.job_process = job, process
                self.jobs.append(job)
                self.log("panel", f"Запущено: {mode} · {settings.face if settings.selection == 'face' else settings.edge}")
                threading.Thread(target=self._read, args=(process, "weld", job), daemon=True).start()
                return dict(job)

    def _stop_job(self):
        if self.job_process and self.job_process.poll() is None:
            if not self.job.get("stopping"):
                self.job["stopping"] = True
                self.job["phase"] = "Отмена операции…"
                self.job_process.send_signal(signal.SIGINT)
                self.log("panel", "Запрошена отмена текущей операции")

    def stop(self):
        with self.commands, self.lock:
            self._stop_job()

    def replace_workpiece(self, data):
        if not isinstance(data, dict) or set(data)-{'object', 'shape'}:
            raise ValueError('Ожидаются ID объекта и форма заготовки')
        shape = data.get('shape')
        if shape not in ('box', 'cylinder', 'prism'):
            raise ValueError('Выберите параллелепипед, цилиндр или треугольную призму')
        object_id = Settings.parse({'object': data.get('object', 'welding_workpiece')}).object
        with self.commands:
            with self.lock:
                if not self.status()['health']['moveit']:
                    raise ValueError('Сначала запустите сцену MoveIt/RViz')
                current = self.job_process
                if current and current.poll() is None:
                    if self.job['mode'] != 'preview':
                        raise ValueError('Нельзя менять заготовку во время планирования или движения')
                    self._stop_job()
            if current and current.poll() is None:
                try:
                    current.wait(timeout=8)
                except subprocess.TimeoutExpired:
                    raise ValueError('Предпросмотр ещё останавливается') from None
            argv = [sys.executable, str(self.executable.with_name('add_workpiece.py')),
                    '--id', object_id, '--frame', self.profile.frame, '--center', '-1', '0', '0.1',
                    '--clear-markers']
            if shape == 'prism':
                mesh = Path(get_package_share_directory('ur10_isaac_moveit_config'))/'config/triangular_prism.json'
                argv += ['--shape', 'mesh', '--mesh-json', str(mesh)]
            else:
                argv += ['--shape', shape, '--dimensions']
                argv += ['0.3', '0.3', '0.2'] if shape == 'box' else ['0.2', '0.15']
            try:
                result = subprocess.run(argv, cwd=self.workspace, capture_output=True, text=True,
                                        errors='replace', timeout=80)
            except subprocess.TimeoutExpired:
                raise ValueError('MoveIt не подтвердил замену заготовки вовремя; проверьте сцену') from None
            if result.returncode:
                detail = (result.stderr or result.stdout).strip()[-1500:]
                self.log('scene', detail)
                raise ValueError('Не удалось заменить заготовку: '+detail)
            with self.lock:
                self.object_edges.pop(object_id, None)
                self.telemetry['objects'] = [o for o in self.telemetry['objects'] if o['id'] != object_id]
            self.log('scene', f'Заготовка {object_id} заменена: {shape}; центр (-1, 0, 0.1) м')
            return {'object': object_id, 'shape': shape}

    def scene(self, start, rviz=True):
        with self.commands, self.lock:
            if start:
                if self.profile.real:
                    raise ValueError('Запустите физический JAKA отдельно: ./run_real.sh <IP>. Панель подключится автоматически')
                if self.scene_process and self.scene_process.poll() is None or self.status()["health"]["moveit"]:
                    raise ValueError("Сцена уже запущена; панель подключается к ней автоматически")
                self.scene_process = subprocess.Popen(
                    ["bash", str(self.workspace / "run_isaac_with_object.sh"), f"rviz:={'true' if rviz else 'false'}"]
                    + (['torch:=true'] if self.profile.tip == 'welding_tcp' else []),
                    cwd=self.workspace, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                    text=True, errors="replace", start_new_session=True)
                self.scene_stopping = False
                threading.Thread(target=self._read, args=(self.scene_process, "scene"), daemon=True).start()
            else:
                if self.job_process and self.job_process.poll() is None:
                    raise ValueError("Сначала остановите текущую операцию")
                if self.scene_process and self.scene_process.poll() is None and not self.scene_stopping:
                    self.scene_stopping = True
                    self.scene_process.send_signal(signal.SIGINT)

    def report(self, job_id):
        if not re.fullmatch(r"[0-9a-f]{32}", job_id):
            raise ValueError("Неизвестный отчёт")
        files = sorted((self.report_root / job_id).glob("*.json"))
        if not files:
            raise ValueError("Отчёт ещё не готов")
        return files[-1].read_bytes()

    def close(self):
        self.stop()
        if self.job_process:
            try:
                self.job_process.wait(timeout=12)
            except subprocess.TimeoutExpired:
                raise RuntimeError("Отмена не завершилась. Панель остаётся доступной для контроля") from None
        self.scene(False)
        if self.scene_process:
            try:
                self.scene_process.wait(timeout=12)
            except subprocess.TimeoutExpired:
                self.log("panel", "ROS launch ещё завершает сцену")
