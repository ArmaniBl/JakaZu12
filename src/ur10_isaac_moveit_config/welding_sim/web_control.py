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

import numpy as np

from .geometry import FACES, face_contour, select_edges
from .geometry import WorkpieceGeometry
from .profiles import ISAAC
from .orientation import tilted_orientation
from .table_limits import JAKA_TABLE_TOP_M
from .workpiece_model import preview_object, WORKPIECE_CATALOG, CATALOG_PREFIX


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


def weld_contours(geometry):
    if geometry.selectable_seams:
        return geometry.selectable_seams
    if geometry.shape != 'mesh':
        return tuple(geometry.contours)
    significant = []
    for name, contour in geometry.contours.items():
        length = contour.length
        if length < .018:
            continue
        vertices = contour.vertices[:-1] - contour.vertices[0]
        area = abs(np.cross(vertices, np.roll(vertices, -1, axis=0)).sum(axis=0) @ contour.outward) / 2
        # Area divided by perimeter distinguishes full loops from long, narrow
        # facets on tessellated cylinders. Units are meters.
        if area >= 25e-6 and area / length >= .0012:
            significant.append(name)
    return tuple(significant)


def geometry_payload(edges, settings, demo=False, *, profile=ISAAC):
    from .probing import probe_contacts
    geometry = edges if isinstance(edges, WorkpieceGeometry) else WorkpieceGeometry(
        'box', edges, {f: face_contour(edges, f) for f in FACES})
    edges = geometry.edges if geometry.shape != 'mesh' else []
    faces = [(name,c) for name,c in geometry.contours.items() if c.kind not in ('hole','weld')]
    choices = weld_contours(geometry)
    face = settings.face if settings.face in choices else max(choices, default=None,
        key=lambda name:(float(geometry.contours[name].outward[2]),geometry.contours[name].kind!='hole',geometry.contours[name].length))
    edge = settings.edge if settings.edge in {e.id for e in edges} else edges[0].id if edges else None
    selected = geometry.select([edge])[0] if settings.selection == 'edge' and edge else (
        geometry.contour(face) if face else None)
    seam, tool, orientation = [], [], []
    vertical_clearance_m = profile.path_vertical_clearance(selected)
    if selected is not None:
        seam, tool, orientation = selected.path(settings.begin, settings.end, settings.offset,
                                               settings.step, math.radians(settings.roll))
        seam, tool = seam.tolist(), (tool + (0., 0., vertical_clearance_m)).tolist()
        orientation = tilted_orientation(orientation, settings.tilt, settings.tilt_direction).tolist()
    return {"demo": demo, "shape": geometry.shape, 'selected_face': face, 'selected_edge': edge,
            "faces": [{"id": name, "vertices": c.vertices[:-1].tolist(),
                       "loops": [v[:-1].tolist() for v in c.loops] if c.loops is not None else [c.vertices[:-1].tolist()],
                       "normal": c.outward.tolist(), "length": c.length} for name,c in faces],
            "seams": [{"id": name,"label": c.label or name,"kind": c.kind,
                       "face_id": c.face_id or name,"vertices": c.vertices.tolist(),
                       "normal": c.outward.tolist(),"length_m": c.length}
                       for name,c in geometry.contours.items() if name in choices],
            "selected_surface": (geometry.contours[face].face_id or face) if face else None,
            "edges": [{"id": e.id, "faces": e.faces, "start": e.start.tolist(),
                       "end": e.end.tolist(), "length": e.length,
                       'vertices': (e.vertices.tolist() if hasattr(e, 'vertices') else [e.start.tolist(), e.end.tolist()])} for e in edges],
            "probe_contacts": probe_contacts(geometry, selected, seam) if selected is not None else [],
            "seam": seam, "tool": tool, "orientation": orientation,
            "vertical_clearance_m": vertical_clearance_m,
            "normal": selected.outward.tolist() if selected is not None else [],
            "length": selected.length*(settings.end-settings.begin) if selected is not None else 0.}


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
        self.scan_progress = dict(state='WAITING', observed_unix=0.)
        self.scene_stopping = False
        self.telemetry = {"isaac": False, "feedback": False, "moveit": False,
                          "controller": False, "objects": [], "error": None,
                          "robot": {"available": False, "frame": "world", "points": [],
                                    "reason": "Ожидаем состояние робота"}}
        self.object_edges = {}
        self.scene_received = False
        self.scene_generation = 0
        self.workcell_objects = []
        self.workcell_revision = 0
        self.animation_pose = None
        self.last_workcell_pose = None
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
                health.update(isaac=False, feedback=False, moveit=False, controller=False)
                health["robot"] = {"available": False, "frame": "world", "points": [],
                                   "reason": "Нет актуальной связи с ROS"}
            from .scan_timestamp import latest_update
            return {"health": health, 'robot_profile': self.profile.payload(), "job": dict(self.job) if self.job else None,
                    'workpiece_catalog': [{'shape': CATALOG_PREFIX + key, 'label': f'{name} · {size}'}
                                          for key, name, size in WORKPIECE_CATALOG],
                    "scan_updated_unix": latest_update(self.workspace),
                    "scan_progress": dict(self.scan_progress),
                    "history": [dict(job) for job in reversed(self.jobs[-12:])],
                    "scene": {"owned": bool(self.scene_process and self.scene_process.poll() is None),
                              "stopping": self.scene_stopping, "generation": self.scene_generation},
                    "logs": [entry for entry in self.logs if entry["seq"] > since], "sequence": self.sequence}

    def geometry(self, settings):
        with self.lock:
            live = self.object_edges.get(settings.object)
            generation = self.scene_generation
            preset = next((o.get('preset') for o in self.telemetry['objects'] if o['id'] == settings.object), None)
            scene_received = self.scene_received or generation > 0
            current = self.telemetry['moveit'] and time.monotonic() - self.last_telemetry <= 4
        if live is not None:
            result = geometry_payload(live, settings, profile=self.profile)
            result.update(source='moveit' if current else 'last_moveit', scene_generation=generation,
                          object_id=settings.object, preset=preset)
            return result
        if scene_received:
            return {'source': 'awaiting_scene', 'scene_generation': generation}
        if settings.object == 'welding_workpiece':
            path=self.workspace/'src/jaka_zu12_moveit_config/config/workpiece_model.json'
            if path.is_file():
                from .runner import object_geometry
                stamp=(str(path),path.stat().st_mtime_ns)
                cached=getattr(self,'cached_cad_geometry',None)
                if cached is None or cached[0]!=stamp:
                    model=json.loads(path.read_text())
                    preview = preview_object(model)
                    if self.profile.real:
                        preview.pose.position.z += JAKA_TABLE_TOP_M
                    edges = object_geometry(preview, self.profile.frame)
                    self.cached_cad_geometry=(stamp,model,edges)
                else:
                    _,model,edges=cached
                result=geometry_payload(edges, settings, demo=True, profile=self.profile)
                result.update(source='cached_cad',part_name=model['part_name'])
                return result
        return {'source': 'awaiting_scene', 'scene_generation': generation}

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
                    elif "Executing " in text and ":probe:" in text:
                        point, motion = text.split(':probe:', 1)[1].split(':', 1)
                        action = {'approach': 'подвод', 'contact': 'касание по CAD',
                                  'standoff': 'контрольная точка с зазором', 'retract': 'отвод'}.get(motion)
                        job['phase'] = f'Прощупывание {point} · {action}' if action else 'Прощупывание'
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
                    if settings.selection == 'face':
                        if settings.face not in weld_contours(geometry):
                            raise ValueError('На этой модели нет доступных маршрутов сварки')
                    else:
                        if geometry.shape == 'mesh':
                            raise ValueError('На этой модели нет доступных маршрутов сварки')
                        geometry.select([settings.edge])
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
                if self.profile.real and mode in ('plan', 'execute'):
                    argv += ['--probe-standoff']
                if self.profile.real:
                    argv += ['--use-scene-workpiece']
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
        catalog = next((key for key, _, _ in WORKPIECE_CATALOG if shape == CATALOG_PREFIX + key), None)
        if catalog is None:
            raise ValueError('Выберите CAD-заготовку из каталога')
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
            center = ('-0.225', '-0.74', str(JAKA_TABLE_TOP_M)) if self.profile.real else ('-1', '0', '0')
            argv = [sys.executable, str(self.executable.with_name('add_workpiece.py')),
                    '--id', object_id, '--frame', self.profile.frame, '--center', *center,
                    '--clear-markers', '--catalog-model', catalog]
            if self.profile.real:
                argv += ['--require-jaka-idle']
            try:
                result = subprocess.run(argv, cwd=self.workspace, capture_output=True, text=True,
                                        errors='replace', timeout=90)
            except subprocess.TimeoutExpired:
                raise ValueError('MoveIt не подтвердил замену заготовки вовремя; проверьте сцену') from None
            if result.returncode:
                detail = (result.stderr or result.stdout).strip()[-1500:]
                self.log('scene', detail)
                raise ValueError('Не удалось заменить заготовку: '+detail)
            if self.profile.real:
                scan_directory = self.workspace / 'scans'
                try:
                    scan_directory.mkdir(exist_ok=True)
                    request_path = scan_directory / 'workpiece-scan-request'
                    temporary = request_path.with_suffix('.tmp')
                    temporary.write_text(f'MoveIt-{catalog}-{uuid.uuid4().hex}')
                    temporary.replace(request_path)
                except OSError as error:
                    self.log('scanner', f'Деталь заменена, но запрос перезапуска сканера не сохранён: {error}')
            with self.lock:
                self.scene_generation += 1
                self.workcell_objects = [obj for obj in self.workcell_objects if obj['id'] != object_id]
                self.workcell_revision += 1
                generation = self.scene_generation
                self.object_edges.pop(object_id, None)
                self.telemetry['objects'] = [o for o in self.telemetry['objects'] if o['id'] != object_id]
            self.log('scene', f'Заготовка {object_id} заменена: {shape}, позиция ({", ".join(center)}) м')
            return {'object': object_id, 'shape': shape, 'scene_generation': generation}

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
