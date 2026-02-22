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

# Wiring/motor parameters
invert_B = True
control_dt = 0.02
key_timeout = 0.18
throttle_hold_window = 0.60
turn_throttle_grace = 0.80
idle_throttle_clear = 1.20
deadband = 1e-4

# Throttle-hold speed levels (PWM)
speed_levels = [0.38, 0.62, 0.92]  # L1, L2, L3
level_step_seconds = 2.0

# Turning
# Matched to the simple test script feel (outer~0.65, inner~0.35).
turn_yield_ratio = 0.54  # inner wheel speed = base * ratio when turning while moving
pivot_pwm = 0.68

# Safety
direction_change_delay = 0.20
brake_latch = False
direction_block_until = 0.0

# Key state
key_state = {"w": False, "s": False, "a": False, "d": False}
key_last_seen = {"w": 0.0, "s": 0.0, "a": 0.0, "d": 0.0}

# Throttle hold state
hold_direction = 0   # 1=forward, -1=backward, 0=idle
hold_start_time = 0.0
active_level_index = 0
last_drive_direction = 0
last_throttle_seen = 0.0
last_throttle_direction = 0


def clamp(value, low, high):
    return max(low, min(high, value))


def get_pressed_key(timeout=0.01):
    dr, _, _ = select.select([sys.stdin], [], [], timeout)
    if dr:
        return sys.stdin.read(1).lower()
    return None


def key_active(key, now, hold_window=0.0):
    if key_state[key]:
        return True
    if hold_window <= 0.0:
        return False
    return (now - key_last_seen[key]) <= hold_window


def set_motor_direction(left_forward, right_forward):
    motorA_in1.on() if left_forward else motorA_in1.off()
    motorA_in2.off() if left_forward else motorA_in2.on()

    effective_right_forward = (right_forward != invert_B)
    motorB_in3.on() if effective_right_forward else motorB_in3.off()
    motorB_in4.off() if effective_right_forward else motorB_in4.on()


def command_motor_signed(left_cmd, right_cmd):
    left_cmd = clamp(left_cmd, -1.0, 1.0)
    right_cmd = clamp(right_cmd, -1.0, 1.0)

    if abs(left_cmd) < deadband and abs(right_cmd) < deadband:
        stop_motors()
        return

    left_forward = left_cmd >= 0.0
    right_forward = right_cmd >= 0.0
    set_motor_direction(left_forward, right_forward)
    motorA_pwm.value = abs(left_cmd)
    motorB_pwm.value = abs(right_cmd)


def stop_motors():
    motorA_pwm.value = 0.0
    motorB_pwm.value = 0.0
    motorA_in1.off()
    motorA_in2.off()
    motorB_in3.off()
    motorB_in4.off()


def apply_brake():
    # Dynamic brake for immediate stop.
    motorA_pwm.value = 0.0
    motorB_pwm.value = 0.0
    motorA_in1.on()
    motorA_in2.on()
    motorB_in3.on()
    motorB_in4.on()


def reset_speed_level(now):
    global hold_start_time, active_level_index
    hold_start_time = now
    active_level_index = 0


def update_speed_level(now):
    global active_level_index
    elapsed = max(0.0, now - hold_start_time)
    computed_index = int(elapsed // level_step_seconds)
    active_level_index = min(len(speed_levels) - 1, computed_index)
    return speed_levels[active_level_index]


def resolve_motion_command(throttle_direction, a, d, base_pwm):
    # Returns signed wheel commands in [-1, 1].
    moving_forward = throttle_direction > 0
    moving_backward = throttle_direction < 0
    throttle_active = moving_forward or moving_backward
    turn_left = a and not d
    turn_right = d and not a

    # Movement with differential-yield turn.
    if throttle_active:
        sign = 1.0 if moving_forward else -1.0
        outer = base_pwm
        inner = base_pwm * turn_yield_ratio

        # Fixed differential steering: slow wheel on turn side.
        if turn_left:
            left = inner
            right = outer
        elif turn_right:
            left = outer
            right = inner
        else:
            left = outer
            right = outer

        return left * sign, right * sign, (1 if moving_forward else -1)

    # Pivot in place.
    if turn_left:
        return -pivot_pwm, pivot_pwm, 0
    if turn_right:
        return pivot_pwm, -pivot_pwm, 0

    return 0.0, 0.0, 0


def release_timed_out_keys(now):
    for key in key_state:
        if key_state[key] and (now - key_last_seen[key]) > key_timeout:
            key_state[key] = False


fd = None
old_settings = None
try:
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    tty.setcbreak(fd)
    log.info("terminal initialized")
except Exception as exc:
    log.error("terminal init failed: %s", exc)

try:
    log.info("robot3 started - W/S move, A/D turn, SPACE brake, Q quit")
    while True:
        now = time.time()

        # Read all available keys.
        while True:
            key = get_pressed_key(timeout=0.001)
            if not key:
                break
            if key == "q":
                raise KeyboardInterrupt()
            if key == " ":
                brake_latch = True
                apply_brake()
                hold_direction = 0
                last_drive_direction = 0
                reset_speed_level(now)
                log.info("BRAKE engaged")
                continue
            if key in key_state:
                key_state[key] = True
                key_last_seen[key] = now

        release_timed_out_keys(now)

        w = key_active("w", now, throttle_hold_window)
        s = key_active("s", now, throttle_hold_window)
        a = key_active("a", now)
        d = key_active("d", now)

        # Brake latch: require releasing all drive keys before any new motion.
        if brake_latch:
            apply_brake()
            if not (w or s or a or d):
                brake_latch = False
                log.info("BRAKE released; waiting for new command")
            time.sleep(control_dt)
            continue

        # Determine throttle direction.
        if w and not s:
            throttle_direction_raw = 1
        elif s and not w:
            throttle_direction_raw = -1
        else:
            throttle_direction_raw = 0

        if throttle_direction_raw != 0:
            last_throttle_seen = now
            last_throttle_direction = throttle_direction_raw

        # Prevent accidental pivot from missed key-repeat while turning.
        # If a turn key is active and we still remember a throttle direction,
        # keep moving-turn mode instead of dropping to pivot.
        if (
            throttle_direction_raw == 0
            and (a or d)
            and last_throttle_direction != 0
        ):
            throttle_direction = last_throttle_direction
        else:
            throttle_direction = throttle_direction_raw
            # Clear throttle memory only after sustained true idle.
            if (
                throttle_direction == 0
                and not (a or d)
                and (now - last_throttle_seen) > idle_throttle_clear
            ):
                last_throttle_direction = 0

        # Speed level logic.
        if throttle_direction == 0:
            hold_direction = 0
            reset_speed_level(now)
            base_pwm = speed_levels[0]
        else:
            if throttle_direction != hold_direction:
                hold_direction = throttle_direction
                reset_speed_level(now)
            base_pwm = update_speed_level(now)

        # Resolve wheel commands based on control table.
        left_cmd, right_cmd, drive_direction = resolve_motion_command(
            throttle_direction, a, d, base_pwm
        )

        # Safety: delay on forward<->reverse changes.
        opposite_change = (
            drive_direction != 0
            and last_drive_direction != 0
            and drive_direction != last_drive_direction
        )
        if opposite_change:
            direction_block_until = now + direction_change_delay
            last_drive_direction = 0
            stop_motors()
            time.sleep(control_dt)
            continue

        if now < direction_block_until:
            stop_motors()
            time.sleep(control_dt)
            continue

        # If no command, stay stopped.
        if abs(left_cmd) < deadband and abs(right_cmd) < deadband:
            stop_motors()
            last_drive_direction = 0
        else:
            command_motor_signed(left_cmd, right_cmd)
            if drive_direction != 0:
                last_drive_direction = drive_direction

        time.sleep(control_dt)

except KeyboardInterrupt:
    pass
finally:
    stop_motors()
    hold_direction = 0
    active_level_index = 0
    if old_settings is not None and fd is not None:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    log.info("robot3 exited safely")
