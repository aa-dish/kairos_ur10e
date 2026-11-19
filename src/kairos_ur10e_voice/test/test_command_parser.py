import math

import pytest

from kairos_ur10e_voice.command_parser import (JOINT_ELBOW, JOINT_LIFT, JOINT_PAN,
                                               JOINT_WRIST3, VOCAB, parse)


def one(text):
    acts = parse(text)
    assert len(acts) == 1, acts
    return acts[0]


def test_stop():
    assert one('stop').kind == 'stop'
    assert one('robot emergency stop').kind == 'stop'
    assert parse('move forward then stop')[0].kind == 'stop'


def test_forward_default_and_distance():
    a = one('move forward')
    assert a.kind == 'base_move' and a.direction == (1.0, 0.0, 0.0)
    assert a.mode == 'distance' and a.amount == pytest.approx(1.0)
    a = one('go forward two meters')
    assert a.amount == pytest.approx(2.0)
    a = one('drive backward fifty centimeters')
    assert a.direction == (-1.0, 0.0, 0.0) and a.amount == pytest.approx(0.5)
    a = one('move forward half a meter')
    assert a.amount == pytest.approx(0.5)
    a = one('move forward one point five meters')
    assert a.amount == pytest.approx(1.5)


def test_strafe_and_diagonal():
    a = one('move left one meter')
    assert a.direction == (0.0, 1.0, 0.0)
    a = one('move right')
    assert a.direction == (0.0, -1.0, 0.0)
    a = one('move forward and left')
    assert a.direction[0] == pytest.approx(math.sqrt(0.5))
    assert a.direction[1] == pytest.approx(math.sqrt(0.5))


def test_time_based():
    a = one('move forward for five seconds')
    assert a.mode == 'time' and a.amount == pytest.approx(5.0)


def test_turn():
    a = one('turn left')
    assert a.direction == (0.0, 0.0, 1.0) and a.amount == pytest.approx(math.radians(90))
    a = one('turn right forty five degrees')
    assert a.direction == (0.0, 0.0, -1.0) and a.amount == pytest.approx(math.radians(45))
    a = one('turn around')
    assert a.amount == pytest.approx(math.pi)
    a = one('rotate one hundred eighty degrees clockwise')
    assert a.direction[2] == -1.0 and a.amount == pytest.approx(math.pi)


def test_arm_poses():
    assert one('arm home').pose == 'home'
    assert one('move the arm to the ready position').pose == 'home'
    assert one('arm vertical').pose == 'up'
    assert one('arm forward').pose == 'forward'
    assert one('go home').kind == 'arm_pose'
    assert one('wave').kind == 'arm_wave'
    assert one('arm wave hello').kind == 'arm_wave'


def test_arm_deltas():
    a = one('rotate the arm left')
    assert a.kind == 'arm_delta' and a.joint == JOINT_PAN and a.delta > 0
    a = one('swing arm right thirty degrees')
    assert a.joint == JOINT_PAN and a.delta == pytest.approx(-math.radians(30))
    a = one('raise the arm')
    assert a.joint == JOINT_LIFT and a.delta < 0
    a = one('lower arm')
    assert a.joint == JOINT_LIFT and a.delta > 0
    a = one('extend the arm')
    assert a.joint == JOINT_ELBOW and a.delta < 0
    a = one('twist the wrist')
    assert a.joint == JOINT_WRIST3


def test_gripper():
    for text, frac in [('open the gripper', 0.0), ('close the gripper', 1.0),
                       ('grab', 1.0), ('release', 0.0), ('open', 0.0), ('close', 1.0),
                       ('open the hand', 0.0), ('close gripper halfway', 0.5),
                       ('half open the gripper', 0.5), ('please grasp', 1.0)]:
        a = one(text)
        assert a.kind == 'gripper', text
        assert a.amount == pytest.approx(frac), text
    assert parse('gripper')[0].kind == 'unknown'


def test_gripper_in_sequence():
    acts = parse('move forward then close the gripper then raise the arm then open the gripper')
    assert [a.kind for a in acts] == ['base_move', 'gripper', 'arm_delta', 'gripper']
    assert [a.amount for a in acts if a.kind == 'gripper'] == [1.0, 0.0]


def test_speed():
    assert one('faster').factor > 1
    assert one('slow down').factor < 1
    assert one('normal speed').kind == 'speed_reset'


def test_sequence():
    acts = parse('move forward two meters then turn left then raise the arm')
    assert [a.kind for a in acts] == ['base_move', 'base_move', 'arm_delta']


def test_unknown():
    assert one('banana').kind == 'unknown'
    assert parse('') == []


@pytest.mark.parametrize('text', [
    'robot move forward two meters', 'turn right ninety degrees',
    'move the arm to the home position', 'rotate the arm left thirty degrees',
    'move forward one point five meters then turn around',
    'move left for ten seconds', 'extend the arm', 'twist the wrist', 'normal speed',
    'quarter turn left', 'move forward half a meter', 'turn counterclockwise',
    'raise the arm then lower the arm', 'wave hello', 'one hundred eighty degrees',
    'open the gripper', 'close the gripper halfway', 'grab', 'release', 'open the hand',
    'move forward then close the gripper then raise the arm',
])
def test_examples_inside_vocab(text):
    """Every example utterance must be producible under the recogniser grammar."""
    for tok in text.split():
        assert tok in VOCAB, tok
