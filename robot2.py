from gpiozero import PWMLED, DigitalOutputDevice
import time, sys, tty, termios, select
import logging

logging.basicConfig(level=logging.INFO, format='[%(levelname)s] %(message)s')
log = logging.getLogger("robot2")

# ----------------------------
# Motor pins
# ----------------------------
motorA_in1 = DigitalOutputDevice(17)
motorA_in2 = DigitalOutputDevice(27)
motorB_in3 = DigitalOutputDevice(22)
motorB_in4 = DigitalOutputDevice(23)

motorA_pwm = PWMLED(12)  # ENA
motorB_pwm = PWMLED(13)  # ENB

# ----------------------------
# Car states
# ----------------------------
STATE_STANDING = 1
STATE_MOVING = 2
STATE_DECELERATING = 3
STATE_TURN_IN_PLACE = 4
STATE_TURN_WHILE_MOVING = 5
STATE_BRAKE = 6

state = STATE_STANDING

# ----------------------------
# Car state
# ----------------------------
base_speed = 0.0
base_direction = 0

min_torque = 0.35
acceleration = 0.08      # faster acceleration for responsive feel
deceleration = 0.04      # faster deceleration
max_speed = 1.0

startup_boost_steps = 2
startup_boost_increment = 0.12
startup_counter = 0

turn_left = 0.0
turn_right = 0.0
turn_increment = 0.25    # faster buildup for stronger turns
turn_decay = 0.04        # slightly faster decay
turn_max = 0.95

turn_in_place_speed = 0.8 * max_speed  # increased from 0.7
moving_turn_scale = 1.0   # changed from 0.85 — full turn effect while moving

invert_B = True
brake_active = False

key_check_interval = 0.02
eps = 1e-4

# Key state tracker with timestamps
key_state = {'w': False, 's': False, 'a': False, 'd': False}
key_last_seen = {'w': 0.0, 's': 0.0, 'a': 0.0, 'd': 0.0}
key_timeout = 0.15  # if no repeat in 0.15s, treat as released

def get_pressed_key(timeout=0.01):
    """Check if key pressed, return key or None"""
    dr, dw, de = select.select([sys.stdin], [], [], timeout)
    if dr:
        return sys.stdin.read(1).lower()
    return None

def apply_brake():
    """Apply fast braking"""
    motorA_pwm.value = 0
    motorB_pwm.value = 0
    motorA_in1.on()
    motorA_in2.on()
    motorB_in3.on()
    motorB_in4.on()

def update_motors_from_state():
    """Set GPIO outputs according to state"""
    global base_speed, base_direction, turn_left, turn_right, state

    if state == STATE_BRAKE or brake_active:
        apply_brake()
        return

    turn_strength = turn_left - turn_right
    ts = max(-turn_max, min(turn_max, turn_strength))

    # Pivot when in-place turning state or when speed ~ 0 and significant turn requested
    if (state == STATE_TURN_IN_PLACE) or (base_speed <= eps and abs(ts) > 0.01):
        spin_speed = turn_in_place_speed * abs(ts)
        if ts > 0:  # pivot left
            dirA = False
            dirB = True
        else:       # pivot right
            dirA = True
            dirB = False
        motorA_in1.on() if dirA else motorA_in1.off()
        motorA_in2.off() if dirA else motorA_in2.on()
        eff_dir_B = (dirB != invert_B)
        motorB_in3.on() if eff_dir_B else motorB_in3.off()
        motorB_in4.off() if eff_dir_B else motorB_in4.on()
        motorA_pwm.value = spin_speed
        motorB_pwm.value = spin_speed
        log.debug("pivot: speed=%.2f ts=%.2f", spin_speed, ts)
        return

    # Moving: apply differential smoothly — reduce inner wheel but keep outer wheel strong
    scale = moving_turn_scale if base_speed > eps else 1.0

    left_speed = base_speed
    right_speed = base_speed

    if ts > 0.01:
        # left turn: reduce left, keep right at or slightly above base_speed
        reduction = ts * scale * 0.6  # gentler reduction so car doesn't lose momentum
        left_speed = max(0.1 * base_speed, base_speed * (1.0 - reduction))
        right_speed = base_speed  # keep right wheel at full speed
    elif ts < -0.01:
        # right turn: reduce right, keep left at or slightly above base_speed
        reduction = abs(ts) * scale * 0.6
        right_speed = max(0.1 * base_speed, base_speed * (1.0 - reduction))
        left_speed = base_speed  # keep left wheel at full speed

    dirA = True if base_direction >= 0 else False
    dirB = True if base_direction >= 0 else False

    motorA_in1.on() if dirA else motorA_in1.off()
    motorA_in2.off() if dirA else motorA_in2.on()

    eff_dir_B = (dirB != invert_B)
    motorB_in3.on() if eff_dir_B else motorB_in3.off()
    motorB_in4.off() if eff_dir_B else motorB_in4.on()

    motorA_pwm.value = left_speed
    motorB_pwm.value = right_speed
    log.debug("state=%d speed=%.2f dir=%d ts=%.2f left=%.2f right=%.2f", 
              state, base_speed, base_direction, ts, left_speed, right_speed)

# ----------------------------
# Initialize terminal
# ----------------------------
try:
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    tty.setcbreak(fd)
    log.info("terminal initialized in raw mode")
except Exception as e:
    log.error("failed to init terminal: %s", e)
    fd = None
    old_settings = None

try:
    log.info("robot2 started - Hold WASD to drive, SPACE to brake, Q to quit")
    
    while True:
        now = time.time()
        
        # Non-blocking key input: read all available keys this frame
        while True:
            key = get_pressed_key(timeout=0.001)
            if not key:
                break
            if key == 'q':
                raise KeyboardInterrupt()
            elif key == ' ':
                brake_active = True
                state = STATE_BRAKE
                base_speed = 0.0
                base_direction = 0
                turn_left = 0.0
                turn_right = 0.0
                apply_brake()
                log.info("BRAKE engaged")
            elif key in key_state:
                key_state[key] = True
                key_last_seen[key] = now
                log.debug("key down: %s", key)

        # Timeout keys if no repeat received (terminal key-repeat gap)
        for key in key_state:
            if key_state[key] and (now - key_last_seen[key]) > key_timeout:
                key_state[key] = False
                log.debug("key timeout (released): %s", key)

        w = key_state['w']
        s = key_state['s']
        a = key_state['a']
        d = key_state['d']

        # Release brake if any movement key is pressed
        if brake_active:
            if any((w, s, a, d)):
                brake_active = False
                state = STATE_STANDING
                log.info("brake released")
            else:
                apply_brake()
                time.sleep(key_check_interval)
                continue

        # STATE DECISION
        if (not w and not s) and (a or d) and base_speed <= eps:
            state = STATE_TURN_IN_PLACE
        elif (a or d) and base_speed > eps:
            state = STATE_TURN_WHILE_MOVING
        elif (w or s) and not (a or d):
            state = STATE_MOVING
        elif not (w or s or a or d):
            if base_speed > eps:
                state = STATE_DECELERATING
            else:
                state = STATE_STANDING
        
        # STATE BEHAVIOR
        if state == STATE_MOVING:
            if w:
                if base_speed <= eps:
                    base_speed = min_torque
                    startup_counter = startup_boost_steps
                elif startup_counter > 0:
                    base_speed = min(max_speed, base_speed + startup_boost_increment)
                    startup_counter -= 1
                else:
                    base_speed = min(max_speed, base_speed + acceleration)
                base_direction = 1
            elif s:
                if base_speed <= eps:
                    base_speed = min_torque
                    startup_counter = startup_boost_steps
                elif startup_counter > 0:
                    base_speed = min(max_speed, base_speed + startup_boost_increment)
                    startup_counter -= 1
                else:
                    base_speed = min(max_speed, base_speed + acceleration)
                base_direction = -1

            if not a:
                turn_left = max(0.0, turn_left - turn_decay)
            if not d:
                turn_right = max(0.0, turn_right - turn_decay)

        elif state == STATE_TURN_WHILE_MOVING:
            # Maintain forward/back speed while turning
            if w:
                if base_speed <= eps:
                    base_speed = min_torque
                    startup_counter = startup_boost_steps
                elif startup_counter > 0:
                    base_speed = min(max_speed, base_speed + startup_boost_increment)
                    startup_counter -= 1
                else:
                    base_speed = min(max_speed, base_speed + acceleration)
                base_direction = 1
            elif s:
                if base_speed <= eps:
                    base_speed = min_torque
                    startup_counter = startup_boost_steps
                elif startup_counter > 0:
                    base_speed = min(max_speed, base_speed + startup_boost_increment)
                    startup_counter -= 1
                else:
                    base_speed = min(max_speed, base_speed + acceleration)
                base_direction = -1

            # Build turn accumulators ONLY while key is held; decay only on release
            if a:
                turn_left = min(turn_max, turn_left + turn_increment)
                turn_right = max(0.0, turn_right - turn_decay * 2)  # kill right turn if left pressed
            else:
                turn_left = max(0.0, turn_left - turn_decay)
            
            if d:
                turn_right = min(turn_max, turn_right + turn_increment)
                turn_left = max(0.0, turn_left - turn_decay * 2)  # kill left turn if right pressed
            else:
                turn_right = max(0.0, turn_right - turn_decay)

        elif state == STATE_DECELERATING:
            startup_counter = 0
            base_speed = max(0.0, base_speed - deceleration)
            if base_speed <= eps:
                base_speed = 0.0
                base_direction = 0

            if a and base_speed > eps:
                state = STATE_TURN_WHILE_MOVING
                turn_left = min(turn_max, turn_left + turn_increment)
            else:
                turn_left = max(0.0, turn_left - turn_decay)

            if d and base_speed > eps:
                state = STATE_TURN_WHILE_MOVING
                turn_right = min(turn_max, turn_right + turn_increment)
            else:
                turn_right = max(0.0, turn_right - turn_decay)

        elif state == STATE_TURN_IN_PLACE:
            base_speed = 0.0
            base_direction = 0
            if a and not d:
                turn_left = min(turn_max, turn_left + turn_increment)
            else:
                turn_left = max(0.0, turn_left - turn_decay)
            if d and not a:
                turn_right = min(turn_max, turn_right + turn_increment)
            else:
                turn_right = max(0.0, turn_right - turn_decay)

            if turn_left <= 0.01 and turn_right <= 0.01:
                state = STATE_STANDING

        elif state == STATE_STANDING:
            base_speed = 0.0
            base_direction = 0
            turn_left = max(0.0, turn_left - turn_decay)
            turn_right = max(0.0, turn_right - turn_decay)
            if turn_left <= 0.01 and turn_right <= 0.01:
                turn_left = 0.0
                turn_right = 0.0

        # Cleanup
        if turn_left < 0.01:
            turn_left = 0.0
        if turn_right < 0.01:
            turn_right = 0.0

        # Update motors
        update_motors_from_state()
        time.sleep(key_check_interval)

except KeyboardInterrupt:
    pass
finally:
    brake_active = False
    base_speed = 0.0
    base_direction = 0
    motorA_pwm.value = 0
    motorB_pwm.value = 0
    motorA_in1.off()
    motorA_in2.off()
    motorB_in3.off()
    motorB_in4.off()
    if old_settings and fd:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    log.info("robot2 exited safely")
