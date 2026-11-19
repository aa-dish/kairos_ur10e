# kairos_ur10e_voice

Voice control of the **real Robotnik RB-Kairos+ (with UR10e) plus a Robotiq 2F-85 gripper** in
Gazebo Harmonic. ROS 2 Jazzy, Ubuntu 24.04 (also works in WSL2).

The robot, controllers and world come from Robotnik's own simulation. This package adds the gripper,
a gripper controller and the speech layer.

| Part | Source |
|---|---|
| Kairos+ base, UR10e mount, sensors, base + arm controllers | `robotnik_description`, `robotnik_simulation` (branch `jazzy-devel`) |
| Robotiq 2F-85 + UR adapter | `ros2_robotiq_gripper` (package `robotiq_description`) |
| Assembly (Kairos+ + UR10e + gripper) | `urdf/rbkairos_plus_gripper.urdf.xacro` |
| Gripper controller | `config/gripper_controller.yaml` (`JointGroupPositionController`) |
| Speech in | `speech_recognizer` node: Vosk (offline) or keyboard |
| Command logic | `command_parser.py` (unit-tested) + `command_interpreter` node |

## Workspace layout (`~/kairos_ur10e/src`)

```
kairos_ur10e_voice/      this package
robotnik_description/    github.com/RobotnikAutomation/robotnik_description   (jazzy-devel)
robotnik_simulation/     github.com/RobotnikAutomation/robotnik_simulation    (jazzy-devel)
ros2_robotiq_gripper/    github.com/PickNikRobotics/ros2_robotiq_gripper      (main)
+ whatever Robotnik's dependency lists name (see below)
```

Robotnik's repositories depend on further packages (for example `robotnik_sensors`,
`robotnik_common`, and the `robotnik_base_control` controller). List them with:

```bash
cd ~/kairos_ur10e/src
cat robotnik_description/robot_description.repo.yaml
ls robotnik_simulation/dependencies && cat robotnik_simulation/dependencies/*
```

Put each repository named there into `src/` (same branch as listed).

## Build

```bash
cd ~/kairos_ur10e
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-up-to kairos_ur10e_voice
source install/setup.bash
```

`--packages-up-to` builds only what this package needs (no navigation / MoveIt).
If colcon says a package is missing, it names it: add that repository to `src/`.

## Check the description before launching

```bash
xacro $(ros2 pkg prefix --share kairos_ur10e_voice)/urdf/rbkairos_plus_gripper.urdf.xacro \
  gazebo_ignition:=true prefix:=robot_ namespace:=robot > /tmp/robot.urdf && echo XACRO_OK
grep -o 'link name="[^"]*tool0"' /tmp/robot.urdf
grep -o 'joint name="[^"]*knuckle_joint"' /tmp/robot.urdf | head -2
```

- The `tool0` link is expected to be `robot_arm_tool0`. If it has another name, pass it
  (without the `robot_` prefix) as `tool_link:=...` in the xacro command, and change the default
  in `urdf/rbkairos_plus_gripper.urdf.xacro`.
- The gripper joint is expected to be `robot_gripper_robotiq_85_left_knuckle_joint`
  (also used in `config/gripper_controller.yaml`).

## Run

```bash
ros2 launch kairos_ur10e_voice sim.launch.py speech_mode:=keyboard   # type commands
ros2 launch kairos_ur10e_voice sim.launch.py                         # microphone (Vosk)
ros2 launch kairos_ur10e_voice sim.launch.py wake_word:=robot        # only "robot ..." sentences
```

If nodes cannot see each other in WSL (no network interface), first run
`export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST ROS_LOCALHOST_ONLY=1` and `sudo ip link set lo multicast on`.

Test without speech (second terminal):

```bash
ros2 topic pub --once /speech/text std_msgs/msg/String "{data: 'close the gripper'}"
```

## Commands

Base (default 1 m / 90 deg; 0.3 m/s, 0.5 rad/s):
- `move forward / backward / left / right [two meters | fifty centimeters | for five seconds]`
- `move forward and left` (diagonal, mecanum base)
- `turn left / right [ninety degrees]`, `turn around`, `rotate clockwise`

Arm (UR10e):
- Poses: `arm home` / `ready position`, `arm vertical`, `arm forward`, `go home`
- Relative: `rotate the arm left/right [thirty degrees]`, `raise / lower the arm`,
  `extend / bend the arm`, `tilt the wrist up/down`, `twist the wrist`
- `wave` / `arm wave hello`

Gripper (Robotiq 2F-85):
- `open the gripper`, `close the gripper`, `grab`, `grasp`, `release`, `open the hand`
- `close the gripper halfway`, `half open the gripper`

General: `stop` (also `halt`, `freeze`, `emergency stop`), `faster`, `slower`, `slow down`,
`normal speed`. Chain with **then**:
`move forward then close the gripper then raise the arm then open the gripper`.

## Topics and controllers (namespace `/robot`)

| What | Name |
|---|---|
| Base velocity | `/robot/mecanum_drive_controller/reference` (TwistStamped) |
| Arm trajectory | `/robot/joint_trajectory_controller/joint_trajectory` |
| Gripper | `/robot/gripper_controller/commands` (Float64MultiArray, 0.0 open ... 0.79 closed) |
| Joint states | `/robot/joint_states` |

## Troubleshooting

- **Other world:** `world_path:=<file>` (Robotnik maze: `$(ros2 pkg prefix --share robotnik_gazebo_ignition)/worlds/demo.world`).
- **Check controllers:** `ros2 control list_controllers -c /robot/controller_manager` should list
  `joint_state_broadcaster`, `robotnik_base_control`, `joint_trajectory_controller` and
  `gripper_controller` as `active`.
- **Gripper controller fails to load:** check the exact error in the launch log. Typical causes:
  wrong joint name (see description check above) or `position_controllers` not installed
  (`ros-jazzy-position-controllers`).
- **Nothing moves, timers idle:** the interpreter uses simulation time. Check
  `ros2 topic hz /clock`. If there is no `/clock`, bridge it:
  `ros2 run ros_gz_bridge parameter_bridge /clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock`.
- **Arm starts in an odd pose:** say `arm home` first.
- **Black or crashing Gazebo window:** `export LIBGL_ALWAYS_SOFTWARE=1`.
- **Microphone:** list devices with `python3 -m sounddevice`; pass `-p device:=<n>` to the recognizer.
  Voice needs `pip install vosk sounddevice` in a venv (`python3 -m venv --system-site-packages
  ~/voice_venv`) and `export PYTHONPATH=$HOME/voice_venv/lib/python3.12/site-packages:$PYTHONPATH`.

## Limits

- Base distances and angles are open loop (speed x time) on a mecanum base: approximate.
- The arm has soft joint limits (`SOFT_LIMITS` in `command_interpreter_node.py`).
  Signs of relative moves assume the UR ready pose; flip them in `command_parser.py` if inverted.
- Vosk uses a restricted vocabulary (`VOCAB` in `command_parser.py`); add words there with new commands.
- Parser tests: `cd kairos_ur10e_voice && python3 -m pytest test`.
