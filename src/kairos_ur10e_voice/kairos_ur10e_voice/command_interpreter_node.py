"""Text command -> robot motion (Robotnik RB-Kairos+ + UR10e + Robotiq 2F-85).

Subscribes : speech/text (String), joint_states (JointState)
Publishes  : base cmd_vel (Twist)            -> robotnik_base_control (holonomic mecanum base)
             arm joint trajectory            -> joint_trajectory_controller (UR10e)
             gripper commands (Float64MultiArray) -> gripper_controller (Robotiq 2F-85)
             speech/feedback (String), also spoken via espeak-ng
Commands in one utterance (separated by "then") are queued and run one after another.
All topic names are parameters; sim.launch.py sets them for the /robot namespace.
"""
import shutil
import subprocess
from collections import deque

import rclpy
from geometry_msgs.msg import Twist, TwistStamped
from rclpy.duration import Duration
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from kairos_ur10e_voice.command_parser import ARM_POSES, parse

JOINT_SUFFIXES = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
                  'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
# Conservative soft limits (rad) so voice commands cannot drive the arm into the base.
SOFT_LIMITS = [(-6.2, 6.2), (-2.6, 0.0), (-2.7, 2.7), (-6.2, 6.2), (-6.2, 6.2), (-6.2, 6.2)]
GRIPPER_CLOSED = 0.79      # rad, left knuckle joint of the Robotiq 2F-85 (0.0 = open)
GRIPPER_TIME = 1.5         # s allowed for a gripper move


class CommandInterpreter(Node):
    def __init__(self):
        super().__init__('command_interpreter')
        self.declare_parameter('base_speed', 0.3)         # m/s
        self.declare_parameter('turn_speed', 0.5)         # rad/s
        self.declare_parameter('arm_speed', 0.6)          # rad/s (largest joint)
        self.declare_parameter('default_distance', 1.0)   # m
        self.declare_parameter('default_angle', 90.0)     # deg
        self.declare_parameter('arm_step_deg', 15.0)
        self.declare_parameter('tts', True)
        self.declare_parameter('cmd_vel_topic', 'cmd_vel')
        self.declare_parameter('cmd_vel_stamped', False)   # True: publish TwistStamped
        self.declare_parameter('joint_states_topic', 'joint_states')
        self.declare_parameter('arm_topic', 'joint_trajectory_controller/joint_trajectory')
        self.declare_parameter('gripper_topic', 'gripper_controller/commands')
        self.declare_parameter('arm_joint_prefix', 'robot_arm_')

        prefix = self.get_parameter('arm_joint_prefix').value
        self.joints = [prefix + s for s in JOINT_SUFFIXES]

        self.stamped = bool(self._p('cmd_vel_stamped'))
        self.cmd_pub = self.create_publisher(TwistStamped if self.stamped else Twist,
                                             self._p('cmd_vel_topic'), 10)
        self.arm_pub = self.create_publisher(JointTrajectory, self._p('arm_topic'), 10)
        self.grip_pub = self.create_publisher(Float64MultiArray, self._p('gripper_topic'), 10)
        self.fb_pub = self.create_publisher(String, 'speech/feedback', 10)
        self.create_subscription(String, 'speech/text', self._on_text, 10)
        self.create_subscription(JointState, self._p('joint_states_topic'), self._on_joints, 10)

        self.joint_pos = {}
        self.arm_target = None
        self.speed_scale = 1.0
        self.queue = deque()
        self.current = None
        self.twist = Twist()
        self.end_time = None
        self.espeak = shutil.which('espeak-ng') or shutil.which('espeak')
        self.create_timer(0.05, self._tick)
        self.get_logger().info('Command interpreter ready. Topic: speech/text')

    # -- I/O ---------------------------------------------------------------
    def _p(self, name):
        return self.get_parameter(name).value

    def _say(self, text):
        self.get_logger().info(f'feedback: {text}')
        self.fb_pub.publish(String(data=text))
        if self._p('tts') and self.espeak:
            subprocess.Popen([self.espeak, text], stdout=subprocess.DEVNULL,
                             stderr=subprocess.DEVNULL)

    def _on_joints(self, msg: JointState):
        for n, p in zip(msg.name, msg.position):
            self.joint_pos[n] = p

    def _send_twist(self, tw):
        if self.stamped:
            msg = TwistStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'robot_base_footprint'
            msg.twist = tw
            self.cmd_pub.publish(msg)
        else:
            self.cmd_pub.publish(tw)

    def _publish_cmd(self):
        self._send_twist(self.twist)

    # -- command intake ----------------------------------------------------
    def _on_text(self, msg: String):
        actions = parse(msg.data, self._p('arm_step_deg'), self._p('default_distance'),
                        self._p('default_angle'))
        if not actions:
            return
        self.get_logger().info(f'"{msg.data}" -> {[a.kind for a in actions]}')
        if actions[0].kind == 'stop':
            self._stop_all()
            self._say('Stopping')
            return
        if actions[0].kind == 'unknown':
            self._say(actions[0].label)
            return
        self.queue.extend(actions)
        self._say('Okay, ' + ', then '.join(a.label for a in actions if a.label))

    def _stop_all(self):
        self.queue.clear()
        self.current = None
        self.end_time = None
        self.twist = Twist()
        self._publish_cmd()
        hold = self._current_positions()
        if hold:
            self.arm_target = hold
            self._send_traj([(hold, 0.2)])

    # -- arm ---------------------------------------------------------------
    def _current_positions(self):
        if all(j in self.joint_pos for j in self.joints):
            return [self.joint_pos[j] for j in self.joints]
        return None

    def _send_traj(self, points):
        traj = JointTrajectory()
        traj.joint_names = list(self.joints)
        for pos, t in points:
            pt = JointTrajectoryPoint()
            pt.positions = [float(x) for x in pos]
            pt.time_from_start = Duration(seconds=t).to_msg()
            traj.points.append(pt)
        self.arm_pub.publish(traj)

    def _clamp(self, pos):
        return [min(max(p, lo), hi) for p, (lo, hi) in zip(pos, SOFT_LIMITS)]

    def _arm_base(self):
        base = self.arm_target or self._current_positions()
        if base is None:
            self._say('The arm is not ready yet')
        return base

    def _move_arm_to(self, target):
        start = self._arm_base()
        if start is None:
            return 0.0
        target = self._clamp(target)
        biggest = max(abs(a - b) for a, b in zip(target, start))
        t = max(1.0, biggest / (self._p('arm_speed') * self.speed_scale))
        self.arm_target = target
        self._send_traj([(target, t)])
        return t

    # -- sequencer ---------------------------------------------------------
    def _start(self, a):
        """Start an action; returns its duration in seconds (0 = instantaneous)."""
        if a.kind == 'speed':
            self.speed_scale = min(max(self.speed_scale * a.factor, 0.25), 3.0)
            self._say(f'Speed {self.speed_scale:.1f} times')
            return 0.0
        if a.kind == 'speed_reset':
            self.speed_scale = 1.0
            return 0.0

        if a.kind == 'base_move':
            dx, dy, dth = a.direction
            lin = self._p('base_speed') * self.speed_scale
            ang = self._p('turn_speed') * self.speed_scale
            tw = Twist()
            if dth != 0.0:
                tw.angular.z = dth * ang
                dur = a.amount if a.mode == 'time' else a.amount / ang
            else:
                tw.linear.x, tw.linear.y = dx * lin, dy * lin
                dur = a.amount if a.mode == 'time' else a.amount / lin
            self.twist = tw
            return dur

        if a.kind == 'gripper':
            self.grip_pub.publish(Float64MultiArray(data=[a.amount * GRIPPER_CLOSED]))
            return GRIPPER_TIME

        if a.kind == 'arm_pose':
            return self._move_arm_to(ARM_POSES[a.pose])

        if a.kind == 'arm_delta':
            base = self._arm_base()
            if base is None:
                return 0.0
            tgt = list(base)
            tgt[a.joint] += a.delta
            return self._move_arm_to(tgt)

        if a.kind == 'arm_wave':
            base = self._arm_base()
            if base is None:
                return 0.0
            pts, t = [], 0.0
            for off in (0.4, -0.4, 0.4, -0.4, 0.0):
                p = list(base)
                p[0] += off
                t += 0.9
                pts.append((self._clamp(p), t))
            self._send_traj(pts)
            return t
        return 0.0

    def _tick(self):
        now = self.get_clock().now()
        if self.current is not None:
            if now >= self.end_time:
                if self.current.kind == 'base_move':
                    self.twist = Twist()
                    self._publish_cmd()
                self.current = None
            elif self.current.kind == 'base_move':
                self._publish_cmd()          # keep refreshing the velocity (controller timeout)
                return
            else:
                return

        if self.current is None and self.queue:
            a = self.queue.popleft()
            dur = self._start(a)
            if dur > 0.0:
                self.current = a
                extra = 0.2 if a.kind != 'base_move' else 0.0
                self.end_time = now + Duration(seconds=dur + extra)
                if a.kind == 'base_move':
                    self._publish_cmd()
        if self.current is None and not self.queue and self.twist != Twist():
            self.twist = Twist()
            self._publish_cmd()


def main(args=None):
    rclpy.init(args=args)
    node = CommandInterpreter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node._send_twist(Twist())
        except Exception:
            pass
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
