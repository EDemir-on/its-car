from gpiozero import PWMLED, DigitalOutputDevice
import time
import sys
import tty
import termios
import select
import logging

logging.basicConfig(level=logging.INFO, format='[%(levelname)s] %(message)s')
log = logging.getLogger("robot3")

# Motor pins
motorA_in1 = DigitalOutputDevice(17)
motorA_in2 = DigitalOutputDevice(27)
motorB_in3 = DigitalOutputDevice(22)
motorB_in4 = DigitalOutputDevice(23)

motorA_pwm = PWMLED(12)
motorB_pwm = PWMLED(13)

# Tunables
max_speed = 1.0
min_pwm = 0.30

accel_rate = 1.20
decel_rate = 1.60
brake_rate = 2.30

turn_accel = 3.00
turn_decay = 4.00
turn_mix = 0.65
pivot_speed = 0.80

deadband = 0.02
control_dt = 0.02

invert_B = True

# Key handling
key_state = {"w": False, "s": False, "a": False, "d": False}
key_last_seen = {"w": 0.0, "s": 0.0, "a": 0.0, "d": 0.0}
key_timeout = 0.15
# Keep throttle intent alive briefly between keyboard repeat events.
throttle_hold_window = 0.30

# Control state
current_speed = 0.0  # -1..1, signed
current_turn = 0.0   # -1..1, signed (left negative, right positive)
brake_active = False


def clamp(value, low, high):
    return max(low, min(high, value))


def move_toward(current, target, step):
    if current < target:
        return min(target, current + step)
    if current > target:
        return max(target, current - step)
    return current


def key_active(key, now, hold_window=0.0):
    if key_state[key]:
        return True
    if hold_window <= 0.0:
        return False
    return (now - key_last_seen[key]) <= hold_window


def get_pressed_key(timeout=0.01):
    dr, _, _ = select.select([sys.stdin], [], [], timeout)
    if dr:
        return sys.stdin.read(1).lower()
    return None


def apply_brake():
    # Dynamic brake by shorting both motor terminals.
    motorA_pwm.value = 0.0
    motorB_pwm.value = 0.0
    motorA_in1.on()
    motorA_in2.on()
    motorB_in3.on()
    motorB_in4.on()


def stop_motors_coast():
    motorA_pwm.value = 0.0
    motorB_pwm.value = 0.0
    motorA_in1.off()
    motorA_in2.off()
    motorB_in3.off()
    motorB_in4.off()


def set_motor_direction(left_forward, right_forward):
    motorA_in1.on() if left_forward else motorA_in1.off()
    motorA_in2.off() if left_forward else motorA_in2.on()

    eff_right_forward = (right_forward != invert_B)
    motorB_in3.on() if eff_right_forward else motorB_in3.off()
    motorB_in4.off() if eff_right_forward else motorB_in4.on()


def signed_to_dir_pwm(value):
    # Convert signed command to motor direction + magnitude with startup floor.
    if abs(value) < deadband:
        return True, 0.0
    direction_forward = value > 0.0
    magnitude = abs(value)
    pwm = min_pwm + (1.0 - min_pwm) * magnitude
    return direction_forward, clamp(pwm, 0.0, 1.0)


def apply_drive(linear_cmd, turn_cmd):
    # Pivot when almost stationary and turning; otherwise blend turn into differential drive.
    if abs(linear_cmd) < deadband and abs(turn_cmd) >= deadband:
        spin = pivot_speed * abs(turn_cmd)
        if turn_cmd > 0.0:  # right pivot
            left = spin
            right = -spin
        else:               # left pivot
            left = -spin
            right = spin
    else:
        left = clamp(linear_cmd + turn_mix * turn_cmd, -1.0, 1.0)
        right = clamp(linear_cmd - turn_mix * turn_cmd, -1.0, 1.0)

    left_dir, left_pwm = signed_to_dir_pwm(left)
    right_dir, right_pwm = signed_to_dir_pwm(right)

    set_motor_direction(left_dir, right_dir)
    motorA_pwm.value = left_pwm
    motorB_pwm.value = right_pwm


# Initialize terminal
try:
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    tty.setcbreak(fd)
    log.info("terminal initialized")
except Exception as exc:
    log.error("terminal init failed: %s", exc)
    fd = None
    old_settings = None


try:
    log.info("robot3 started - WASD drive, SPACE brake hold, Q quit")
    last_t = time.time()

    while True:
        now = time.time()
        dt = clamp(now - last_t, 0.0, 0.08)
        last_t = now

        # Read all available keys
        while True:
            key = get_pressed_key(timeout=0.001)
            if not key:
                break
            if key == "q":
                raise KeyboardInterrupt()
            if key == " ":
                brake_active = True
                current_speed = 0.0
                current_turn = 0.0
                log.info("BRAKE engaged")
                continue
            if key in key_state:
                key_state[key] = True
                key_last_seen[key] = now

        # Release timed-out keys
        for key in key_state:
            if key_state[key] and (now - key_last_seen[key]) > key_timeout:
                key_state[key] = False

        w = key_active("w", now, throttle_hold_window)
        s = key_active("s", now, throttle_hold_window)
        a = key_active("a", now)
        d = key_active("d", now)

        # Keep brake latched until movement input is received.
        if brake_active:
            if w or s or a or d:
                brake_active = False
                log.info("brake released")
            else:
                apply_brake()
                time.sleep(control_dt)
                continue

        # Signed throttle command: forward=+1, backward=-1.
        if w and not s:
            throttle_target = 1.0
        elif s and not w:
            throttle_target = -1.0
        else:
            throttle_target = 0.0

        # Turn target: left=-1, right=+1.
        if d and not a:
            turn_target = 1.0
        elif a and not d:
            turn_target = -1.0
        else:
            turn_target = 0.0

        target_speed = throttle_target * max_speed

        # Faster braking when reversing direction.
        reversing = (target_speed * current_speed) < 0.0
        if abs(target_speed) > abs(current_speed):
            speed_step = accel_rate * dt
        elif reversing:
            speed_step = brake_rate * dt
        else:
            speed_step = decel_rate * dt
        current_speed = move_toward(current_speed, target_speed, speed_step)

        # Turn rises quickly and decays smoothly to center.
        if abs(turn_target) > abs(current_turn):
            turn_step = turn_accel * dt
        else:
            turn_step = turn_decay * dt
        current_turn = move_toward(current_turn, turn_target, turn_step)
        current_turn = clamp(current_turn, -1.0, 1.0)

        # Zero tiny residuals.
        if abs(current_speed) < deadband:
            current_speed = 0.0
        if abs(current_turn) < deadband:
            current_turn = 0.0

        apply_drive(current_speed, current_turn)
        time.sleep(control_dt)

except KeyboardInterrupt:
    pass
finally:
    stop_motors_coast()
    if old_settings and fd:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    log.info("robot3 exited safely")
