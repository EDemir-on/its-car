from gpiozero import PWMLED, DigitalOutputDevice
import time, sys, tty, termios, select
import logging

logging.basicConfig(level=logging.INFO, format='[%(levelname)s] %(message)s')
log = logging.getLogger("robot2")

# Motor pins
motorA_in1 = DigitalOutputDevice(17)
motorA_in2 = DigitalOutputDevice(27)
motorB_in3 = DigitalOutputDevice(22)
motorB_in4 = DigitalOutputDevice(23)

motorA_pwm = PWMLED(12)
motorB_pwm = PWMLED(13)

# States
STATE_STANDING = 1
STATE_MOVING = 2
STATE_TURN_IN_PLACE = 3
STATE_TURN_WHILE_MOVING = 4
STATE_BRAKE = 5

# State and motion
state = STATE_STANDING
base_speed = 0.0
base_direction = 0  # 1=forward, -1=backward, 0=none

turn_left = 0.0
turn_right = 0.0

# Parameters
min_pwm = 0.3
max_speed = 1.0
acceleration = 0.06
deceleration = 0.05
turn_increment = 0.20
turn_decay = 0.06
turn_max = 0.95
turn_in_place_speed = 0.75

invert_B = True
brake_active = False

key_check_interval = 0.02
eps = 1e-4

# Key state
key_state = {'w': False, 's': False, 'a': False, 'd': False}
key_last_seen = {'w': 0.0, 's': 0.0, 'a': 0.0, 'd': 0.0}
key_timeout = 0.15

def get_pressed_key(timeout=0.01):
    """Non-blocking key read"""
    dr, dw, de = select.select([sys.stdin], [], [], timeout)
    if dr:
        return sys.stdin.read(1).lower()
    return None

def apply_brake():
    """Hard brake: zero PWM and short motor terminals"""
    motorA_pwm.value = 0
    motorB_pwm.value = 0
    motorA_in1.on()
    motorA_in2.on()
    motorB_in3.on()
    motorB_in4.on()

def set_motor_direction(left_dir, right_dir):
    """Set motor direction: True=forward, False=backward"""
    motorA_in1.on() if left_dir else motorA_in1.off()
    motorA_in2.off() if left_dir else motorA_in2.on()
    
    eff_right = (right_dir != invert_B)
    motorB_in3.on() if eff_right else motorB_in3.off()
    motorB_in4.off() if eff_right else motorB_in4.on()

def set_motor_speed(left_speed, right_speed):
    """Set motor PWM values"""
    motorA_pwm.value = left_speed
    motorB_pwm.value = right_speed

def update_motors():
    """Compute and apply motor outputs based on current state"""
    global base_speed, base_direction, turn_left, turn_right, state

    if state == STATE_BRAKE or brake_active:
        apply_brake()
        return

    # Compute turn strength
    turn_strength = turn_left - turn_right
    ts = max(-turn_max, min(turn_max, turn_strength))

    # Pivot in place (only in STATE_TURN_IN_PLACE)
    if state == STATE_TURN_IN_PLACE:
        spin_speed = turn_in_place_speed * abs(ts)
        left_dir = (ts > 0)  # ts > 0 = left turn means left motor backward
        right_dir = not (ts > 0)  # right motor forward
        set_motor_direction(not left_dir, not right_dir)
        set_motor_speed(spin_speed, spin_speed)
        log.debug("TURN_IN_PLACE: ts=%.2f spin=%.2f", ts, spin_speed)
        return

    # For moving states: set direction and apply differential
    left_dir = (base_direction > 0)
    right_dir = (base_direction > 0)
    
    left_speed = base_speed
    right_speed = base_speed

    # Apply turn differential
    if state == STATE_TURN_WHILE_MOVING:
        if ts > 0.01:  # turning left
            left_speed = max(min_pwm, base_speed * (1.0 - ts * 0.5))
            right_speed = base_speed
        elif ts < -0.01:  # turning right
            right_speed = max(min_pwm, base_speed * (1.0 - abs(ts) * 0.5))
            left_speed = base_speed

    set_motor_direction(left_dir, right_dir)
    set_motor_speed(left_speed, right_speed)
    log.debug("state=%d speed=%.2f dir=%d ts=%.2f L=%.2f R=%.2f", 
              state, base_speed, base_direction, ts, left_speed, right_speed)

# Initialize terminal
try:
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    tty.setcbreak(fd)
    log.info("terminal initialized")
except Exception as e:
    log.error("terminal init failed: %s", e)
    fd = None
    old_settings = None

try:
    log.info("robot2 started - WASD to drive, SPACE to brake, Q to quit")
    
    while True:
        now = time.time()
        
        # Read all available keys
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
                log.info("BRAKE engaged")
            elif key in key_state:
                key_state[key] = True
                key_last_seen[key] = now

        # Timeout keys if no repeat received
        for key in key_state:
            if key_state[key] and (now - key_last_seen[key]) > key_timeout:
                key_state[key] = False

        w = key_state['w']
        s = key_state['s']
        a = key_state['a']
        d = key_state['d']

        # Release brake on any movement input
        if brake_active:
            if any((w, s, a, d)):
                brake_active = False
                state = STATE_STANDING
                log.info("brake released")
            else:
                apply_brake()
                time.sleep(key_check_interval)
                continue

        # STATE MACHINE
        # From STANDING
        if state == STATE_STANDING:
            if w or s:
                state = STATE_MOVING
                base_direction = 1 if w else -1
                base_speed = min_pwm
            elif (a or d) and not (w or s):
                state = STATE_TURN_IN_PLACE

        # From MOVING
        elif state == STATE_MOVING:
            if w:
                base_direction = 1
                base_speed = min(max_speed, base_speed + acceleration)
            elif s:
                base_direction = -1
                base_speed = min(max_speed, base_speed + acceleration)
            
            # Transition to turn-while-moving if A or D pressed
            if (a or d) and not (w or s):
                state = STATE_TURN_WHILE_MOVING
            # Decelerate if no movement keys
            elif not (w or s):
                base_speed = max(0.0, base_speed - deceleration)
                if base_speed <= eps:
                    state = STATE_STANDING
                    base_speed = 0.0
                    base_direction = 0

        # From TURN_WHILE_MOVING
        elif state == STATE_TURN_WHILE_MOVING:
            if w:
                base_direction = 1
                base_speed = min(max_speed, base_speed + acceleration)
            elif s:
                base_direction = -1
                base_speed = min(max_speed, base_speed + acceleration)
            elif not (w or s):
                # Continue turn but decelerate base speed
                base_speed = max(0.0, base_speed - deceleration)
                if base_speed <= eps:
                    state = STATE_STANDING
                    base_speed = 0.0
                    base_direction = 0
            
            # Exit turn if neither A nor D pressed
            if not (a or d):
                if base_speed > eps and (w or s):
                    state = STATE_MOVING
                elif base_speed <= eps:
                    state = STATE_STANDING

        # From TURN_IN_PLACE
        elif state == STATE_TURN_IN_PLACE:
            # Transition to turn-while-moving if W or S pressed
            if w or s:
                state = STATE_TURN_WHILE_MOVING
                base_direction = 1 if w else -1
                base_speed = min_pwm
            # Exit to standing if no turn keys
            elif not (a or d):
                state = STATE_STANDING
                turn_left = 0.0
                turn_right = 0.0

        # Update turn accumulators
        if a and not d:
            turn_left = min(turn_max, turn_left + turn_increment)
            turn_right = max(0.0, turn_right - turn_decay * 2)
        else:
            turn_left = max(0.0, turn_left - turn_decay)

        if d and not a:
            turn_right = min(turn_max, turn_right + turn_increment)
            turn_left = max(0.0, turn_left - turn_decay * 2)
        else:
            turn_right = max(0.0, turn_right - turn_decay)

        # Cleanup small values
        if turn_left < 0.01:
            turn_left = 0.0
        if turn_right < 0.01:
            turn_right = 0.0

        update_motors()
        time.sleep(key_check_interval)

except KeyboardInterrupt:
    pass
finally:
    motorA_pwm.value = 0
    motorB_pwm.value = 0
    motorA_in1.off()
    motorA_in2.off()
    motorB_in3.off()
    motorB_in4.off()
    if old_settings and fd:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    log.info("robot2 exited safely")
