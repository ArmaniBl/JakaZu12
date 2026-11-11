# Isaac Sim / UR10 / welding

Рабочая папка: `/home/arman/jaka_zu12_welding`, ветка `welding`,
origin: https://github.com/ArmaniBl/JakaZu12.git.
Isaac Sim работает на Windows 11, ROS 2 Jazzy и MoveIt — в WSL2 Ubuntu 24.04.
На этой машине обнаружен Isaac Sim `6.0.0-rc.22`.

## Запуск в WSL

```bash
cd ~/jaka_zu12_welding
# Зависимости на этой машине уже установлены.
# На новой машине с установленным ROS 2 Jazzy:
./install_dependencies.sh
./build.sh
./run_isaac.sh
```

Сначала откройте сцену UR10 и включите Play в Isaac Sim.
`./run_sim.sh` — сокращение для того же запуска.
Без RViz: `./run_isaac.sh rviz:=false`.
Отдельный запуск с готовой заготовкой: `./run_isaac_with_object.sh`.
Он добавляет в сцену MoveIt оранжевый объект `welding_workpiece` размером
`0.30 × 0.30 × 0.20 м` с центром `(-1.0, 0.0, 0.10)` относительно `world`.
Объект занимает X `[-1.15, -0.85]`, Y `[-0.15, 0.15]`, Z `[0.0, 0.20]` м.
Он виден в RViz и учитывается при планировании коллизий.
Это объект сцены планирования: ROS-запуск не создаёт USD-прим в Windows Isaac Sim.
Обычный `./run_isaac.sh` запускает сцену без добавления заготовки.
В RViz выберите группу `ur_manipulator`, сначала Plan, затем Execute.
Скорость и ускорение планирования по умолчанию ограничены коэффициентом 0.05.
Прерывание запуска — Ctrl+C. Журналы и снимки конфигурации находятся в `diagnostics/`.

В другом WSL-терминале:

```bash
cd ~/jaka_zu12_welding
source setup_isaac.bash
ros2 control list_controllers
ros2 topic info /joint_states -v
ros2 topic info /joint_command -v
```

Контроллер `ur10_controller` должен иметь состояние `active`.
Action выполнения: `/ur10_controller/follow_joint_trajectory`.
Пакеты и SDK физического JAKA сохранены в исходниках, но запуск Isaac их не использует.
Для возвращения к исходной конфигурации JAKA/Humble используйте отдельную рабочую копию `main`.

## Модель и интерфейс

Шесть суставов Isaac подтверждены по фактическому `/joint_states`:
`shoulder_pan_joint`, `shoulder_lift_joint`, `elbow_joint`,
`wrist_1_joint`, `wrist_2_joint`, `wrist_3_joint`.
Вместо переименования `joint_1..joint_6` используется официальная модель `ur_description`, тип `ur10`.
Кинематика JAKA и UR10 не считается тождественной: планирование и проверка коллизий выполняются по UR10.
Для переноса сварочных траекторий на JAKA понадобятся его собственная кинематика и калибровка TCP/базы.
Сварочная горелка, TCP, детали и препятствия сцены ещё не описаны в этой конфигурации.

| Топик | Направление | Тип |
| --- | --- | --- |
| `/joint_states` | Isaac → ROS | `sensor_msgs/msg/JointState` |
| `/joint_command` | ROS → Isaac | `sensor_msgs/msg/JointState` |

Связь с ros2_control выполняет стандартный плагин
`joint_state_topic_hardware_interface/JointStateTopicSystem`.
`joint_state_broadcaster`, mock hardware и драйвер JAKA в этом запуске отсутствуют:
единственный источник `/joint_states` — Isaac.

При первоначальной настройке сцена не публиковала `/clock`, а штампы `/joint_states` содержали время симуляции.
Поэтому `use_sim_time=false`, и небольшой узел переиздаёт **только полученные** состояния
с текущим временем WSL в `/ur10/joint_states` для MoveIt и TF.
Без этого MoveIt считает состояния устаревшими и отказывает в Execute.
Исходный `/joint_states` не изменяется. Во время исполнения оставляйте Play включённым:
это управление по настенным часам, без синхронизации времени при паузе симулятора.

После запуска MoveIt появятся его служебные топики (`tf`, `planning_scene`,
`display_planned_path`, состояния контроллера и другие). Они нужны стеку планирования.
При первоначальной проверке после остановки запуска и с работающей сценой оставались только `/joint_states`,
`/joint_command`, `/rosout`, `/parameter_events`.
При вечерней повторной проверке в сцене уже был включён издатель `/clock`
`_Graph_ROS_Clock_PublishClock`. Он принадлежит отдельному графу Isaac, а не старому интерфейсу.
Текущая конфигурация продолжает работать по настенным часам и от наличия `/clock` не зависит.
`/joint_command` виден в списке топиков и при отсутствии издателя, поскольку Isaac на него подписан.

## Найденные лишние топики и исправление Windows

Диагностика 5 ноября 2026 года показала:

| Лишний топик | Узел | Причина |
| --- | --- | --- |
| `/goal_pose` | `isaac_ros_interface` | Издатель старого примера MoveIt |
| `/joint_commands` | `isaac_ros_interface` | Подписчик старого примера; имя во множественном числе |

Источник: `C:\Users\user\Documents\Kit\shared\exts\IsaacMoveitExtension\Isaac_MoveIt_Extension_python\ros_interface.py`.
Этот интерфейс создавался из `scenario.py` при инициализации расширения.
Проверка с `--no-daemon` подтвердила, что это были живые endpoints, а не кэш ros2 daemon.

Изменён внешний Windows-файл `scenario.py`: `_setup_ros_interface()` теперь возвращает управление
без создания старого интерфейса, если переменная `ISAAC_ENABLE_LEGACY_MOVEIT_INTERFACE` не равна `1`.
Резервная копия: `scenario.py.bak_welding_20261105` в том же каталоге.
При горячей перезагрузке старый оставшийся узел был явно уничтожен;
после этого отсутствие обоих лишних топиков проверено заново.
Это локальное изменение Windows находится за пределами Git-репозитория.
При переустановке расширения его нужно повторить. Для восстановления старого примера
задайте `ISAAC_ENABLE_LEGACY_MOVEIT_INTERFACE=1` перед запуском Isaac.
OmniGraph публикации `/joint_states` и подписки `/joint_command` сохранён.

## Сеть

Существующая связь Windows ↔ WSL проверена фактическим обменом сообщениями.
Сохранён режим WSL NAT и существующий профиль `$HOME/.ros/fastdds.xml`
с Windows-пиром `172.17.224.1`; `setup_isaac.bash` использует его, если файл существует.
Обе стороны используют Fast DDS, `ROS_DOMAIN_ID=0`.
Системные настройки сети и Windows Firewall не изменялись.
При изменении IP виртуального адаптера WSL обновите адрес пира в этом профиле;
актуальный Windows-шлюз виден в `ip route show default`.
Новый пустой терминал Isaac должен использовать его внутренние библиотеки ROS 2 Jazzy.

Официальные материалы:
[Windows/WSL и ROS 2](https://docs.isaacsim.omniverse.nvidia.com/6.1.0/installation/install_ros_other_platforms.html),
[плагин JointStateTopicSystem](https://control.ros.org/jazzy/doc/topic_based_hardware_interfaces/doc/index.html).

## Проверка

На работающей сцене выполнены сборка, запуск MoveIt и активация контроллера.
Через `/plan_kinematic_path` построена траектория поворота `wrist_3_joint` на 0.05 рад,
затем выполнена через MoveIt `/execute_trajectory`. Аналогично выполнен возврат.
Оба планирования и исполнения вернули `MoveItErrorCodes.SUCCESS`.
Ошибка обратной связи шестого сустава: 0.0002 рад после поворота и 0.0013 рад после возврата.
Результат сохранён локально в `diagnostics/integration_result.json`.
После точечного обновления `ros-jazzy-rclcpp`, `ros-jazzy-moveit-ros-planning`
и `ros-jazzy-moveit-ros-move-group` движение и возврат проверены повторно:
оба успешны, ошибки 0.0004 и 0.0005 рад соответственно.
Эта проверка подтверждает обмен и исполнение короткой траектории;
сварочная траектория и соответствие всей геометрии USD модели ещё не проверялись.

RViz проверен через WSLg: модель загружается, группа `ur_manipulator` доступна.
При завершении по Ctrl+C воспроизводится segfault установленного MoveIt 2.12.4
в деструкторе `TrajectoryExecutionManager`/`rclcpp::CallbackGroup`.
Точечное обновление трёх пакетов выше его не устранило. Движение и возврат работают,
процессы после остановки завершаются, но завершение `move_group` пока не является чистым.
Повторная передача SIGINT дочерним узлам из `log_run.py` устранена отдельно.
Лог последней проверки: `diagnostics/2026-11-05_19-55-01_isaac_yzbryu14/console.log`.
