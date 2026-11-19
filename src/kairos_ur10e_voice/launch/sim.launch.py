"""Gazebo (Harmonic) simulation of the real Robotnik RB-Kairos+ with UR10e and a Robotiq 2F-85,
controlled by voice.

It re-uses Robotnik's own bringup (robotnik_simulation_bringup/bringup_complete.launch.py) with
navigation, localization, RViz and MoveIt switched off, and with our robot description
(Kairos+ + UR10e + gripper). It then adds a controller helper (see CONTROLLER_SCRIPT) and the
voice nodes.
"""
import os
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (AppendEnvironmentVariable, DeclareLaunchArgument, ExecuteProcess,
                            IncludeLaunchDescription)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

ROBOT_ID = 'robot'   # Robotnik default namespace; frames/joints are prefixed with "robot_"

# Robotnik's own spawner starts the base/arm controllers WITHOUT their parameters on some
# setups (the controller then fails with "Wrong traction/steering declaration"). This helper
# waits for Robotnik's spawner to finish, and if the base controller is not active it reloads
# the base + arm controllers with the (resolved) parameter file. It then adds the gripper.
CONTROLLER_SCRIPT = r'''
CM="$1"; BASE_YAML="$2"; GRIP_YAML="$3"; MEC_YAML="$4"
sleep 10
for i in $(seq 1 120); do
  pgrep -f "[c]ontroller_manager/spawner.*unload-on-kill" >/dev/null || break
  sleep 2
done
# Robotnik's own spawner dies on its base controller; the stock base controller is left unconfigured
# (it is never used). Make sure the broadcaster is up, then add arm, mecanum base and gripper.
spawn() {  # spawn <attempts> <spawner args...>
  n="$1"; shift
  for k in $(seq 1 "$n"); do
    ros2 run controller_manager spawner "$@" -c "$CM" --controller-manager-timeout 60 --service-call-timeout 60 && return 0
    echo "[controllers] retry $k: $*"; sleep 3
  done
  return 1
}
echo "[controllers] joint_state_broadcaster + arm"
spawn 3 joint_state_broadcaster
spawn 3 joint_trajectory_controller -p "$BASE_YAML"
echo "[controllers] mecanum base controller"
ros2 param set "$CM" mecanum_drive_controller.type mecanum_drive_controller/MecanumDriveController --no-daemon
spawn 3 mecanum_drive_controller -p "$MEC_YAML"
echo "[controllers] gripper controller"
ros2 param set "$CM" gripper_controller.type position_controllers/JointGroupPositionController --no-daemon
spawn 3 gripper_controller -p "$GRIP_YAML"
echo "[controllers] done"
'''


def resolved_base_yaml():
    """Robotnik's controller profile with its $(var frame_prefix) placeholder filled in."""
    profile = os.path.join(get_package_share_directory('robotnik_gazebo_ignition'),
                           'config', 'profile', 'rbkairos', 'rbkairos_plus_ros2_control.yaml')
    with open(profile) as f:
        text = f.read().replace('$(var frame_prefix)', f'{ROBOT_ID}_')
    out = os.path.join(tempfile.gettempdir(), 'kairos_ur10e_rbkairos_plus_resolved.yaml')
    with open(out, 'w') as f:
        f.write(text)
    return out


def generate_launch_description():
    pkg = get_package_share_directory('kairos_ur10e_voice')
    bringup = get_package_share_directory('robotnik_simulation_bringup')

    xacro_file = os.path.join(pkg, 'urdf', 'rbkairos_plus_gripper.urdf.xacro')
    gripper_cfg = os.path.join(pkg, 'config', 'gripper_controller.yaml')
    base_cfg = os.path.join(pkg, 'config', 'base_controller.yaml')

    args = [
        DeclareLaunchArgument('speech', default_value='true',
                              description='Start the speech recognizer node'),
        DeclareLaunchArgument('speech_mode', default_value='mic',
                              description='mic (Vosk) or keyboard (type commands)'),
        DeclareLaunchArgument('vosk_model', default_value=os.path.expanduser(
            '~/vosk-model-small-en-us-0.15'), description='Path to the Vosk model folder'),
        DeclareLaunchArgument('wake_word', default_value='',
                              description='Optional wake word, e.g. "robot"'),
        DeclareLaunchArgument('use_gui', default_value='true',
                              description='Show the Gazebo window'),
        DeclareLaunchArgument('low_performance_simulation', default_value='true',
                              description='Lighter simulation (recommended in WSL / VMs)'),
        DeclareLaunchArgument('world_path', default_value=os.path.join(pkg, 'worlds', 'empty.world'),
                              description='World file (default: empty ground, no walls)'),
    ]

    # Robotnik bringup: world + robot (Kairos+ / UR10e / gripper from our xacro) + controllers
    robotnik = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup, 'launch', 'bringup_complete.launch.py')),
        launch_arguments={
            'robot_id': ROBOT_ID,
            'robot': 'rbkairos',
            'robot_model': 'rbkairos_plus',
            'robot_xacro_path': xacro_file,
            'arm_type': 'ur10e',
            'use_gui': LaunchConfiguration('use_gui'),
            'low_performance_simulation': LaunchConfiguration('low_performance_simulation'),
            'world_path': LaunchConfiguration('world_path'),
            'use_rviz': 'false',
            'run_localization': 'false',
            'run_navigation': 'false',
            'run_moveit': 'false',
            'run_laser_filters': 'false',
        }.items())

    controllers = ExecuteProcess(
        cmd=['bash', '-c', CONTROLLER_SCRIPT, 'setup_controllers',
             f'/{ROBOT_ID}/controller_manager', resolved_base_yaml(), gripper_cfg, base_cfg],
        output='screen')

    interpreter = Node(
        package='kairos_ur10e_voice', executable='command_interpreter',
        parameters=[{
            'use_sim_time': True,
            'cmd_vel_topic': f'/{ROBOT_ID}/mecanum_drive_controller/reference',
            'cmd_vel_stamped': True,
            'joint_states_topic': f'/{ROBOT_ID}/joint_states',
            'arm_topic': f'/{ROBOT_ID}/joint_trajectory_controller/joint_trajectory',
            'gripper_topic': f'/{ROBOT_ID}/gripper_controller/commands',
            'arm_joint_prefix': f'{ROBOT_ID}_arm_',
        }],
        output='screen')

    recognizer = Node(
        package='kairos_ur10e_voice', executable='speech_recognizer',
        condition=IfCondition(LaunchConfiguration('speech')),
        parameters=[{'mode': LaunchConfiguration('speech_mode'),
                     'model_path': LaunchConfiguration('vosk_model'),
                     'wake_word': LaunchConfiguration('wake_word')}],
        output='screen', emulate_tty=True)

    # Let Gazebo find the Robotiq meshes (package://robotiq_description/... -> model://...)
    robotiq_share = os.path.dirname(get_package_share_directory('robotiq_description'))
    gz_resources = AppendEnvironmentVariable('GZ_SIM_RESOURCE_PATH', robotiq_share)

    return LaunchDescription(args + [gz_resources, robotnik, controllers,
                                     interpreter, recognizer])
