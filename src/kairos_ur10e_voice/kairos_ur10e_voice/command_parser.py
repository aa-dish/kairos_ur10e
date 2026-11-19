"""Pure-python (no ROS) natural-language command parser.

Turns a recognised utterance such as "move forward two meters then turn left ninety
degrees" into a list of Action objects. Keeping this free of ROS makes it easy to unit-test
and to reuse with another recogniser (Whisper, cloud ASR, keyboard, ...).
"""
import math
import re
from dataclasses import dataclass
from typing import List, Optional, Tuple

# ---------------------------------------------------------------------------
# Arm presets (UR10e joint order: pan, lift, elbow, wrist1, wrist2, wrist3)
# ---------------------------------------------------------------------------
ARM_POSES = {
    'home':    [0.0, -1.57, 1.57, -1.57, -1.57, 0.0],
    'up':      [0.0, -1.57, 0.0, -1.57, 0.0, 0.0],
    'forward': [0.0, -1.00, 1.00, -1.57, -1.57, 0.0],
}

JOINT_PAN, JOINT_LIFT, JOINT_ELBOW, JOINT_WRIST1, JOINT_WRIST2, JOINT_WRIST3 = range(6)

DEFAULT_ARM_STEP_DEG = 15.0
DEFAULT_DISTANCE_M = 1.0
DEFAULT_ANGLE_DEG = 90.0
MAX_DISTANCE_M = 10.0

# Words the speech recogniser is allowed to output (used as a Vosk grammar).
VOCAB = """
robot stop halt freeze abort emergency cancel
move go drive forward forwards ahead straight back backward backwards reverse
left right turn rotate spin pivot face around clockwise counterclockwise anticlockwise
strafe and then next after that afterwards the a an to by for please position pose
meter meters metre metres centimeter centimeters centimetre centimetres
degree degrees second seconds half quarter point hundred
zero one two three four five six seven eight nine ten eleven twelve thirteen fourteen
fifteen sixteen seventeen eighteen nineteen twenty thirty forty fifty sixty seventy
eighty ninety
arm manipulator wrist shoulder elbow raise lift lower up down upward upwards downward
downwards extend straighten stretch unfold bend retract fold contract twist roll tilt
home ready rest stow default vertical upright front wave hello greet
faster slower speed slow normal
gripper hand claw open close grab grasp grip release halfway fully
""".split()


@dataclass
class Action:
    kind: str                       # base_move | stop | arm_pose | arm_delta | arm_wave
    #                                 | speed | speed_reset | unknown
    direction: Tuple[float, float, float] = (0.0, 0.0, 0.0)   # unit (x, y, yaw) for base
    amount: float = 0.0             # metres | radians | seconds (see mode)
    mode: str = ''                  # distance | angle | time
    pose: str = ''                  # named arm pose
    joint: int = -1                 # arm joint index for arm_delta
    delta: float = 0.0              # radians for arm_delta
    factor: float = 1.0             # for speed
    text: str = ''                  # original (segment) text
    label: str = ''                 # human readable description for feedback


# ---------------------------------------------------------------------------
# Text normalisation
# ---------------------------------------------------------------------------
_UNITS = {'zero': 0, 'one': 1, 'two': 2, 'three': 3, 'four': 4, 'five': 5, 'six': 6,
          'seven': 7, 'eight': 8, 'nine': 9}
_TEENS = {'ten': 10, 'eleven': 11, 'twelve': 12, 'thirteen': 13, 'fourteen': 14,
          'fifteen': 15, 'sixteen': 16, 'seventeen': 17, 'eighteen': 18, 'nineteen': 19}
_TENS = {'twenty': 20, 'thirty': 30, 'forty': 40, 'fifty': 50, 'sixty': 60,
         'seventy': 70, 'eighty': 80, 'ninety': 90}


def _read_number(toks: List[str], i: int) -> Tuple[int, int]:
    cur, last = 0, None
    while i < len(toks):
        t = toks[i]
        if t in _UNITS and last in (None, 'tens', 'hundred'):
            cur += _UNITS[t]
            last = 'unit'
        elif t in _TEENS and last in (None, 'hundred'):
            cur += _TEENS[t]
            last = 'teen'
        elif t in _TENS and last in (None, 'hundred'):
            cur += _TENS[t]
            last = 'tens'
        elif t == 'hundred' and last == 'unit':
            cur *= 100
            last = 'hundred'
        else:
            break
        i += 1
    return cur, i


def words_to_digits(text: str) -> str:
    toks = text.split()
    out, i = [], 0
    while i < len(toks):
        t = toks[i]
        if t in _UNITS or t in _TEENS or t in _TENS:
            val, i = _read_number(toks, i)
            if i < len(toks) and toks[i] == 'point':
                j, frac = i + 1, ''
                while j < len(toks) and toks[j] in _UNITS:
                    frac += str(_UNITS[toks[j]])
                    j += 1
                if frac:
                    out.append(f'{val}.{frac}')
                    i = j
                    continue
            out.append(str(val))
        else:
            out.append(t)
            i += 1
    return ' '.join(out)


def normalise(text: str) -> str:
    t = text.lower().replace('-', ' ')
    t = re.sub(r'[^a-z0-9. ]', ' ', t)
    t = re.sub(r'(?<!\d)\.|\.(?!\d)', ' ', t)
    t = re.sub(r'\bhalf turn\b', 'turn around', t)
    t = re.sub(r'\bquarter turn\b', 'turn 90 degrees', t)
    t = re.sub(r'\bhalf (?:a |an )?', '0.5 ', t)
    t = words_to_digits(t)
    t = re.sub(r'\b(?:a|an)\s+(?=(?:meter|metre|degree|second|centimeter|centimetre)s?\b)', '1 ', t)
    t = re.sub(r'\s+', ' ', t).strip()
    return t


_QTY_RE = re.compile(
    r'(\d+(?:\.\d+)?)\s*(meters?|metres?|centimeters?|centimetres?|cm|degrees?|deg|'
    r'seconds?|secs?|radians?|m|s)?\b')


def _quantity(text: str) -> Tuple[Optional[float], Optional[str]]:
    m = _QTY_RE.search(text)
    if not m:
        return None, None
    val, unit = float(m.group(1)), (m.group(2) or '')
    if unit.startswith('centi') or unit == 'cm':
        return val / 100.0, 'm'
    if unit.startswith(('meter', 'metre')) or unit == 'm':
        return val, 'm'
    if unit.startswith('deg'):
        return val, 'deg'
    if unit.startswith('rad'):
        return val, 'rad'
    if unit.startswith('sec') or unit == 's':
        return val, 's'
    return val, ''


# ---------------------------------------------------------------------------
# Parser
# ---------------------------------------------------------------------------
_STOP = {'stop', 'halt', 'freeze', 'abort', 'emergency', 'cancel'}
_ARM_CTX = {'arm', 'manipulator', 'wrist', 'shoulder', 'elbow', 'wave', 'hello', 'greet'}
_ROTATE = {'turn', 'rotate', 'spin', 'pivot', 'face'}


def _parse_segment(seg: str, step_deg: float, def_dist: float, def_angle: float
                   ) -> Optional[Action]:
    w = set(seg.split())
    has = lambda *ws: any(x in w for x in ws)  # noqa: E731
    qty, unit = _quantity(seg)

    # ---- speed ----
    if has('faster') or ('speed' in w and has('up')):
        return Action('speed', factor=1.5, text=seg, label='speeding up')
    if has('slower') or ('slow' in w and has('down')):
        return Action('speed', factor=1 / 1.5, text=seg, label='slowing down')
    if 'normal' in w and 'speed' in w:
        return Action('speed_reset', text=seg, label='normal speed')

    # ---- gripper (Robotiq 2F-85): fraction 0.0 = open, 1.0 = closed ----
    opening = has('open', 'release')
    closing = has('close', 'grab', 'grasp', 'grip')
    if opening or closing or has('gripper', 'claw'):
        if opening == closing:          # neither, or contradictory
            return None
        half = has('half', 'halfway') or (qty == 0.5 and unit == '')
        frac = 0.5 if half else (1.0 if closing else 0.0)
        if half:
            label = 'moving the gripper halfway'
        else:
            label = 'closing the gripper' if closing else 'opening the gripper'
        return Action('gripper', amount=frac, text=seg, label=label)

    left, right = has('left'), has('right')
    fwd = has('forward', 'forwards', 'ahead', 'straight') and not has('upright')
    back = has('back', 'backward', 'backwards', 'reverse')
    pose_words = {
        'home': has('home', 'ready', 'rest', 'stow', 'default'),
        'up': has('vertical', 'upright'),
        'forward': has('front') or fwd,
    }
    arm_ctx = bool(w & _ARM_CTX)

    # ---- arm ----
    if not arm_ctx and (pose_words['home'] or pose_words['up']) and not (left or right or back):
        arm_ctx = True   # "go home", "ready position" -> arm
    if arm_ctx:
        if has('wave', 'hello', 'greet'):
            return Action('arm_wave', text=seg, label='waving')

        step = math.radians(step_deg)
        if qty is not None and unit in ('deg', ''):
            step = math.radians(qty)
        elif qty is not None and unit == 'rad':
            step = qty

        if has('extend', 'straighten', 'stretch', 'unfold'):
            return Action('arm_delta', joint=JOINT_ELBOW, delta=-step, text=seg,
                          label='extending the elbow')
        if has('bend', 'retract', 'fold', 'contract'):
            return Action('arm_delta', joint=JOINT_ELBOW, delta=step, text=seg,
                          label='bending the elbow')
        if 'wrist' in w:
            if has('twist', 'roll', 'rotate', 'turn', 'spin') or left or right:
                sign = 1.0 if (left or not right) else -1.0
                return Action('arm_delta', joint=JOINT_WRIST3, delta=sign * step, text=seg,
                              label='rotating the wrist')
            if has('up', 'raise', 'lift', 'tilt', 'upward', 'upwards'):
                return Action('arm_delta', joint=JOINT_WRIST1, delta=step, text=seg,
                              label='tilting the wrist up')
            if has('down', 'lower', 'downward', 'downwards'):
                return Action('arm_delta', joint=JOINT_WRIST1, delta=-step, text=seg,
                              label='tilting the wrist down')
        for name in ('home', 'up', 'forward'):
            if pose_words[name]:
                return Action('arm_pose', pose=name, text=seg, label=f'arm to {name} pose')
        if has('raise', 'lift', 'up', 'upward', 'upwards'):
            return Action('arm_delta', joint=JOINT_LIFT, delta=-step, text=seg,
                          label='raising the arm')
        if has('lower', 'down', 'downward', 'downwards'):
            return Action('arm_delta', joint=JOINT_LIFT, delta=step, text=seg,
                          label='lowering the arm')
        if left or right:
            return Action('arm_delta', joint=JOINT_PAN, delta=step if left else -step,
                          text=seg, label=f'swinging the arm {"left" if left else "right"}')
        return None

    # ---- base rotation ----
    clockwise = has('clockwise')
    ccw = has('counterclockwise', 'anticlockwise')
    if has(*_ROTATE) or clockwise or ccw:
        if qty is not None and unit == 's':
            mode, amount = 'time', qty
        else:
            deg = def_angle
            if has('around'):
                deg = 180.0
            if qty is not None and unit in ('deg', ''):
                deg = qty
            amount, mode = math.radians(min(deg, 720.0)), 'angle'
        sign = 1.0
        if right or (clockwise and not ccw):
            sign = -1.0
        label_dir = 'right' if sign < 0 else 'left'
        return Action('base_move', direction=(0.0, 0.0, sign), amount=amount, mode=mode,
                      text=seg, label=f'turning {label_dir}')

    # ---- base translation ----
    dx = (1.0 if fwd else 0.0) - (1.0 if back else 0.0)
    dy = (1.0 if left else 0.0) - (1.0 if right else 0.0)
    if dx == 0.0 and dy == 0.0:
        return None
    norm = math.hypot(dx, dy)
    dx, dy = dx / norm, dy / norm
    if qty is not None and unit == 's':
        mode, amount = 'time', qty
    else:
        dist = def_dist
        if qty is not None and unit in ('m', ''):
            dist = qty
        mode, amount = 'distance', min(dist, MAX_DISTANCE_M)
    names = []
    if dx > 0:
        names.append('forward')
    if dx < 0:
        names.append('backward')
    if dy > 0:
        names.append('left')
    if dy < 0:
        names.append('right')
    return Action('base_move', direction=(dx, dy, 0.0), amount=amount, mode=mode, text=seg,
                  label='moving ' + ' and '.join(names))


def parse(text: str, arm_step_deg: float = DEFAULT_ARM_STEP_DEG,
          default_distance: float = DEFAULT_DISTANCE_M,
          default_angle: float = DEFAULT_ANGLE_DEG) -> List[Action]:
    """Parse an utterance into a list of Actions (possibly a single 'unknown')."""
    norm = normalise(text)
    if not norm:
        return []
    if set(norm.split()) & _STOP:
        return [Action('stop', text=norm, label='stopping')]

    actions = []
    for seg in re.split(r'\b(?:and then|then|after that|afterwards|next)\b', norm):
        seg = seg.strip()
        if not seg:
            continue
        a = _parse_segment(seg, arm_step_deg, default_distance, default_angle)
        if a is not None:
            actions.append(a)
    return actions or [Action('unknown', text=norm, label='sorry, I did not understand')]
