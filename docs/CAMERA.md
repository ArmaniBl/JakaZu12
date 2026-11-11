# Камера и облако точек: Windows Isaac Sim → WSL2

Рабочая сцена: `C:\Users\user\Desktop\ur10\ur_welding.usd`.
Источник изображения — `/World/Camera`, расположенная в `(-1, 0, 1)` м,
смотрит вниз на заготовку. Разрешение **640 × 480**, облако XYZ содержит
307 200 точек, 3 686 400 байт. Стандартное ограничение — каждый 12-й кадр:
при 60 кадрах/с это примерно 5 публикаций/с; настенная частота зависит от скорости сцены.

## Настройки сцены

- `/Graph/ROS_Camera/RenderProduct.inputs:cameraPrim` → `/World/Camera`.
- `DepthPclPublish`: тип `depth_pcl`, топик `/depth_pcl`, `frameId=sim_camera`.
- `CameraInfoPublish`: топик `/camera_info`, тот же `frameId`.
- Оба helper: `frameSkipCount=11`, `queueSize=1`, `useSystemTime=true`,
  `resetSimulationTimeOnStop=false`. RGB и отдельный depth image остаются выключены.
- `/Graph/WeldingCameraTF` публикует `/tf_static`: `world` → `sim_camera`.
  Перенос `(-1, 0, 1)` м, quaternion XYZW `(1, 0, 0, 0)` переводит оси USD-камеры
  в ROS optical: X вправо, Y вниз, Z вперёд.

TF рассчитан для неподвижной камеры. После изменения её положения запустите
ремонт сцены повторно. Все указанные параметры, включая TF, записаны в USD.
Камера использует системное время в соответствии с текущим режимом роботного
стека `use_sim_time=false`. Топик `/clock` сохраняет время симуляции: сравнивать
его числа напрямую с метками облака нельзя. Измерение абсолютной сетевой
задержки требует отдельной синхронизации часов Windows и WSL.

## Ограничитель публикации в Isaac Sim 6.0.0-rc.22

В этой установленной версии нативный Replicator NodeWriter запускает PointCloud2
через `WriterSyncGate`, обходя старый ограничитель CameraHelper. Поэтому одного
`frameSkipCount` оказалось недостаточно. Поддержка в [camera_repair.py](../isaac/camera_repair.py)
вставляет отдельный `IsaacSimulationGate` перед издателями облака и CameraInfo.
Старые ограничения соответствующего render product выставляются в 1, чтобы не
ограничивать поток дважды. Другие камеры и роботные графы не изменяются.

Временные SDG-узлы создаются в session layer. Фоновая задача восстанавливает
ограничитель при появлении новых издателей, в том числе после Stop/Play.
Задача отменяется при выгрузке расширения.

Поддержка уже установлена в локальное `IsaacMoveitExtension`. Для повторной установки
в PowerShell из корня проекта:

```powershell
powershell.exe -NoProfile -ExecutionPolicy Bypass -File .\isaac\install_camera_support.ps1
```

Скрипт сохраняет резервную копию `extension.py`, устанавливает модуль и добавляет
вызов `start()` при старте расширения и отмену задачи при завершении. При активной
сцене выполняется настройка камеры с предварительным сохранением root/session
layers в `%USERPROFILE%\.ros\camera_backup_YYYYMMDD_HHMMSS`.

Однократный ремонт можно выполнить в Script Editor Isaac Sim:

```python
import asyncio
path = r"\\wsl.localhost\Ubuntu-24.04\home\arman\jaka_zu12_welding\isaac\camera_repair.py"
scope = {}
exec(compile(open(path, encoding="utf-8").read(), path, "exec"), scope)
asyncio.ensure_future(scope["repair"]())
```

## Профили Fast DDS

Ранее через NAT шли UDP-сообщения до 65 500 байт. В захвате наблюдались
неполные наборы RTPS-фрагментов облаков. На обеих сторонах теперь используется
пользовательский UDPv4 transport:

| Параметр | Значение |
| --- | --- |
| `maxMessageSize` | 1400 байт |
| `sendBufferSize` | 16 МиБ |
| `receiveBufferSize` | 16 МиБ |
| `useBuiltinTransports` | false |

Это сохраняет каждый UDP-пакет в пределах обычного MTU и исключает IP-фрагментацию
на этом участке сети. Формат параметров описан в
[документации Fast DDS](https://fast-dds.docs.eprosima.com/en/2.6.x/fastdds/xml_configuration/transports.html).

На Windows `%USERPROFILE%\.ros\write_fastdds_profile.ps1` обновлён:
существующий `IsaacSim_ROS2.bat` продолжает автоматически получать текущий IP WSL
и формировать `%USERPROFILE%\.ros\fastdds.xml`. Копия генератора находится
в [isaac/write_fastdds_profile.ps1](../isaac/write_fastdds_profile.ps1).

В WSL обновлён `~/.ros/fastdds.xml`, список peer-адресов сохранён. Для повторения:

```bash
python3 isaac/configure_wsl_dds.py
sudo install -m 644 isaac/99-welding-dds.conf /etc/sysctl.d/99-welding-dds.conf
sudo sysctl -p /etc/sysctl.d/99-welding-dds.conf
```

Профиль DDS читается при создании процесса: после его изменения следует
перезапустить Isaac Sim и ROS-процессы. Исходные XML-профили обеих сторон и
Windows-генератор сохранены рядом с суффиксом `.bak_camera_20261105`.
Экспериментальные изменения `ipfrag_*` отменены; их увеличение не решило проблему.

## Проверка

Живой просмотр без запуска управления роботом: `./run_camera.sh` в WSL.
Открывается RViz с подпиской `/depth_pcl`, Fixed Frame `world`, сеткой 10 см
и окраской точек по высоте. Мышью можно вращать вид и приближать заготовку.

Нажмите Play в Isaac Sim. В WSL из корня проекта:

```bash
source setup_isaac.bash
python3 isaac/check_camera.py --reliable --cloud-only --seconds 60
```

Проверка не отправляет команды роботу. JSON и последнее облако NPZ сохраняются
в `diagnostics/camera_*/`. Проверяются приём по пятисекундным интервалам,
максимальный разрыв, валидность XYZ, TF на момент облака и размеры верхней
плоскости тестовой заготовки. В RViz задайте Fixed Frame `world`, добавьте
PointCloud2 `/depth_pcl`; для подписки подходит Reliable или Best Effort.

Подтверждённый замер 05.11.2026, 23:14 МСК: **311 облаков за 60 с**, средняя
частота 5,18 Гц, максимальный интервал 0,214 с; 307 200 конечных точек,
TF доступен. Верхняя поверхность имеет X от −1,1501 до −0,8513 м,
Y от −0,1487 до 0,1501 м и Z ≈ 0,2 м.

После окончательной установки модуля проверен и подписчик Best Effort:
152 облака за 30 с, максимальный интервал 0,221 с, TF доступен.
