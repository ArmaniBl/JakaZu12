# RangeVision Spectrum → MoveIt (fixed scanner, JAKA)

## Кратко: порядок работы

Подготовлен импорт **экспортированного облака**, а не прямой захват с USB.
Сканер подключается к Windows, обработка скана выполняется в ScanCenter /
RV 3D Studio. HDMI используется для проектора. Подходящая версия ПО и драйверов
зависит от ревизии Spectrum и лицензии; официальные ссылки ниже.

1. Установить ПО производителя, откалибровать сам сканер его штатной панелью.
2. Жёстко закрепить сканер у стола и измерить соответствующие опорные точки
   в системе экспорта сканера и системе `world` робота.
3. Командой `calibrate` получить преобразование в `world`. Требуются отдельные
   контрольные точки; выдуманная/единичная калибровка не поставляется.
4. Экспортировать облако в ASCII PLY или XYZ/ASC. Размерность указать явно.
5. Командой `publish` показать облако в RViz; командой `compare` сравнить
   выбранную область с текущей заготовкой. Нижняя поверхность стола не должна
   попадать в область сравнения.
6. Проверить сдвиг и качество, затем отдельной командой `apply` переместить
   **модель заготовки**, сохранив её ориентацию. Физический объект и робот
   этой командой не перемещаются.
7. В панели выбрать грань/ребро и построить новый план. Начало движения —
   только существующей отдельной командой исполнения после проверки плана.

Ограничения первой версии: уточняется только перенос до 50 мм, не поворот.
Нужно видеть поверхности, определяющие положение по трём осям; один вид сверху
не даёт достаточных данных. Отображаемое облако само по себе **не становится
препятствием**: коллизии проверяются по модели детали и остальным объектам сцены.
При перемещении сканера или изменении системы координат экспорта калибровать заново.
Фактическая точность и работа со Spectrum пока не проверены: оборудование ещё
не подключено и ПО сканирования не установлено.

## What is implemented

This is an **export-file bridge**, not a USB camera driver or automatic scanning
SDK. Scan in Windows using the vendor's software; export **ASCII PLY** or
headerless XYZ/ASC. WSL transforms the measured points into robot `world`,
publishes `/rangevision/points`, compares them with the actual MoveIt workpiece,
and can translate that workpiece after a separate approval command.

The scanner stays fixed relative to the robot base. Moving the scanner,
changing its calibration/measurement area, or changing the export coordinate
system invalidates the scanner-to-world calibration. Do not let the scanning
software recenter, rotate, scale or independently align each export.

Official downloads/manuals:

- https://rangevision.com/en/support/software/
- https://rangevision.com/en/support/manuals/
- https://support.rangevision.com/

The official site lists RV 3D Studio and ScanCenter NG releases.
Choose software/drivers for the **actual serial number and hardware revision**;
do not assume the current Spectrum 2025 instructions match an older scanner.
USB devices acquire the camera images; the HDMI output drives the structured
light projector. A video feed alone is not a calibrated depth cloud.

## Windows preparation

1. Install the matching RangeVision software and camera drivers from the vendor.
   If the download/license is not available, obtain it from RangeVision support.
2. Follow the model's manual for power, both USB camera connections and HDMI.
   Verify that the software sees both cameras and the projector. Keep the
   cameras on Windows; USB passthrough to WSL is not needed for this workflow.
3. Calibrate the scanner with its supplied calibration board for the selected
   measuring area, focus and exposure. Verify a known physical dimension.
4. Fix the scanner rigidly at the table. Scan only with the robot stationary;
   keep the scanner/stand/cables outside the robot's swept volume and add their
   collision geometry separately if they are within reach.
5. Export an un-recentered cloud as ASCII PLY (XYZ properties, optional colors)
   or whitespace XYZ/ASC (X Y Z in the first three columns, no count header).
   Binary PLY, PTX, proprietary RV3D and a triangle-only STL are not supported.
   Confirm whether coordinates are millimetres or metres.

## WSL setup

```bash
cd ~/welding/JakaZu12
./setup_rangevision.sh
./build.sh --real
mkdir -p scans
```

`setup_rangevision.sh` creates `.venv-rangevision` with SciPy 1.15.3 and inherits
ROS packages. It avoids the local system SciPy/NumPy ABI conflict. If `venv` is
missing, install Ubuntu's `python3-venv`. Scans and the environment are ignored
by Git. `run_rangevision.sh` uses this environment without modifying robot
launch dependencies.

Export on Windows to a folder visible under `/mnt/c/...`, or copy exports to
`scans/`. Publish only finished exports; do not overwrite a file being read.

## Calibrate scanner coordinates to robot world

Scanner factory calibration does **not** supply this transform.
Measure matching stationary fiducial centres in scanner coordinates and robot
`world`. Use a calibrated reference artefact/known robot-world measurements,
not an eyeballed RViz pose. Spread landmarks across the work volume, including
different heights. Welding TCP calibration is a separate prerequisite.

Create `scans/landmarks.json` with these arrays, each row an XYZ measurement:

| JSON field | Units | Purpose |
| --- | --- | --- |
| `scanner_mm` | mm | At least 4 fitting fiducial positions in the exported scanner frame |
| `world_m` | m | Corresponding measured positions in robot world, same order |
| `check_scanner_mm` | mm | At least 4 **different** validation fiducials |
| `check_world_m` | m | Their independently measured world positions |

No example transform is enabled: an identity transform would produce believable
but uncalibrated coordinates. Do not reuse fitting points as validation points.

```bash
./run_rangevision.sh calibrate --landmarks scans/landmarks.json \
  --output scans/calibration.json
```

The fit is a rigid transform (no scaling/reflection). Independent validation
error must be at most 1 mm; that threshold is an initial commissioning gate,
not a certificate of welding accuracy. Recheck reference fiducials after every
scanner movement and before a working session. The program cannot detect a
physically moved scanner from a file alone.

## Display, compare, approve, plan

Start the existing real JAKA launch with automatic enable disabled. Start the
web panel, select the correct **known shape and dimensions**, and keep all
planning/execution jobs stopped while measuring and updating the scene. Do not
send competing goals from RViz, the panel or another client during alignment.
The bridge checks stationary JAKA feedback and MoveIt's execution status before
comparison/update, but is not a controller-level motion interlock.

### 1. Display the measured scan

```bash
./run_rangevision.sh publish --cloud /mnt/c/Scans/part.ply --units mm \
  --calibration scans/calibration.json
```

Keep it running in a separate terminal. RViz's real-robot config includes the
cyan `RangeVision scan` PointCloud2 display with fixed frame `world`. Restart
RViz once after updating this configuration. The table remains grey, the
workpiece orange. This is a static measurement republished for viewing; it is
not a live safety sensor. Stop/restart the publisher after a new export.

**PointCloud2 visualization is not an OctoMap obstacle.** This workflow moves
the known collision model of the workpiece; unmodelled clamps, cables, scanner
stands and other scan points do not automatically become collision objects.

### 2. Compare against the current workpiece

Pick an explicit tight ROI in **world metres**: `xmin xmax ymin ymax zmin zmax`.
It must contain only measured workpiece surfaces, excluding table points,
robot, fixtures and other parts. For the current part near `(0, -0.6)`, determine
bounds from the displayed scan and actual dimensions. Keep the ROI bottom above
the table to exclude its top surface. There is deliberately no default ROI.

```bash
# Replace all six bounds below with measured numeric bounds.
./run_rangevision.sh compare --cloud /mnt/c/Scans/part.ply --units mm \
  --calibration scans/calibration.json \
  --roi XMIN XMAX YMIN YMAX ZMIN ZMAX --report scans/alignment.json
```

The default target is `welding_workpiece`; another ID can be supplied with
`--object`. The actual BOX/CYLINDER/triangle mesh and all local/object poses are
read from MoveIt. `floor` cannot be a target. Comparison never moves the robot
or changes the scene. The report includes translation XYZ in metres, RMSE, P95,
point count, source hashes and whether the correction exceeds 1 mm.

Current registration deliberately supports **translation only**, up to 50 mm.
The workpiece orientation must already match reality. It uses 1 mm model/scan
sampling, requires three independent surface-normal directions, rejects
degenerate observations (e.g. only a flat top), and rejects RMSE/P95 above 2 mm.
A scan of only two box faces is insufficient. Scan the top plus two adjacent
sides, or reposition the fixed scanner and recalibrate it. Repeated features,
symmetry, occlusion and a wrong ROI can still yield an incorrect match; inspect
the overlay. This is local refinement near a known pose, not global object
recognition. Larger shifts/rotation/incorrect dimensions must be corrected and
measured separately before registration.

### 3. Approve the measured displacement

Inspect `scans/alignment.json` and the scan overlay; then explicitly apply:

```bash
./run_rangevision.sh apply --report scans/alignment.json
```

The report expires after 5 minutes. The program reruns comparison, checks scan,
calibration and workpiece hashes, checks stationary feedback/status again, and
updates **only the workpiece translation**, retaining geometry and orientation.
The table and robot are not moved. This command never enables drives or starts
welding. Do not move the physical part after scanning. A saved file's hash does
not prove the physical part is still in the measured place.

### 4. Select a target and create a new welding plan

The web panel will pick up the updated scene. Select the required face/edge,
preview the target, create a **new** plan and inspect the whole torch clearance
before explicit execution. Old plans/previews must be discarded. The existing
runner rejects changed workpiece geometry before execution. After completion,
the real robot retains the configured 25 cm vertical lift behavior.

## Commissioning status

The bridge, local registration and calibration math can be tested without a
scanner. USB enumeration, vendor license/drivers, actual export contents,
fiducial measurements, scanner-to-world accuracy and physical collision-free
operation require the connected Spectrum and a stationary robot. No automatic
capture or welding start is implemented. No claim of physical readiness is made
until those checks pass.

Reproduce offline tests and the fake-scene integration check (no robot driver):

```bash
source setup_isaac.bash
.venv-rangevision/bin/python -m pytest src/ur10_isaac_moveit_config/test/test_scan_alignment.py -q
.venv-rangevision/bin/python src/ur10_isaac_moveit_config/test/scan_bridge_integration.py
```

The integration script uses ROS domain 183 and loopback-only discovery, publishes
fake stationary feedback, and checks compare → explicit apply → stale-report
rejection. Do not use that domain for physical hardware during this test.
