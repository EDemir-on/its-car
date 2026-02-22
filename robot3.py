from gpiozero import PWMLED, DigitalOutputDevice
import logging
import select
import sys
import termios
import threading
import time
import tty

logging.basicConfig(level=logging.INFO, format='[%(levelname)s] %(message)s')
log = logging.getLogger("robot3")

# Motor pins
motorA_in1 = DigitalOutputDevice(17)
motorA_in2 = DigitalOutputDevice(27)
motorB_in3 = DigitalOutputDevice(22)
motorB_in4 = DigitalOutputDevice(23)
motorA_pwm = PWMLED(12)
motorB_pwm = PWMLED(13)

# Hardware config
invert_B = True

# Control timing
control_dt = 0.02
input_poll_dt = 0.005
key_active_window = 0.35
turn_release_grace = 0.30
idle_throttle_clear = 1.20

# Drive behavior
speed_levels = [0.38, 0.62, 0.92]  # Level 1..3
level_step_seconds = 2.0
turn_yield_ratio = 0.54
pivot_pwm = 0.68

# Safety
direction_change_delay = 0.20
deadband = 1e-4


def clamp(value, low, high):
    return max(low, min(high, value))


class InputState:
    def __init__(self):
        self.lock = threading.Lock()
        self.last_seen = {"w": 0.0, "s": 0.0, "a": 0.0, "d": 0.0}
        self.quit_requested = False
        self.brake_pressed = False

    def mark_key(self, key, now):
        with self.lock:
            if key in self.last_seen:
                self.last_seen[key] = now
            elif key == " ":
                self.brake_pressed = True
            elif key == "q":
                self.quit_requested = True

    def snapshot(self):
        with self.lock:
            return (
                dict(self.last_seen),
                self.brake_pressed,
                self.quit_requested,
            )

    def clear_brake(self):
        with self.lock:
            self.brake_pressed = False


def input_reader(stop_event, input_state):
    while not stop_event.is_set():
        dr, _, _ = select.select([sys.stdin], [], [], input_poll_dt)
        if not dr:
            continue
        try:
            ch = sys.stdin.read(1).lower()
        except Exception:
            continue
        input_state.mark_key(ch, time.time())


def set_motor_direction(left_forward, right_forward):
    motorA_in1.on() if left_forward else motorA_in1.off()
    motorA_in2.off() if left_forward else motorA_in2.on()

    effective_right = (right_forward != invert_B)
    motorB_in3.on() if effective_right else motorB_in3.off()
    motorB_in4.off() if effective_right else motorB_in4.on()


def command_motor_signed(left_cmd, right_cmd):
    left_cmd = clamp(left_cmd, -1.0, 1.0)
    right_cmd = clamp(right_cmd, -1.0, 1.0)

    if abs(left_cmd) < deadband and abs(right_cmd) < deadband:
        stop_motors()
        return

    set_motor_direction(left_cmd >= 0.0, right_cmd >= 0.0)
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


def key_is_active(last_seen, key, now):
    return (now - last_seen[key]) <= key_active_window


def resolve_motion(throttle_dir, turn_dir, base_pwm):
    # Returns left_cmd, right_cmd, drive_dir
    if throttle_dir != 0:
        outer = base_pwm
        inner = base_pwm * turn_yield_ratio
        if turn_dir < 0:
            left = inner
            right = outer
        elif turn_dir > 0:
            left = outer
            right = inner
        else:
            left = outer
            right = outer
        sign = 1.0 if throttle_dir > 0 else -1.0
        return left * sign, right * sign, throttle_dir

    # Pivot only when no throttle command is active.
    if turn_dir < 0:
        return -pivot_pwm, pivot_pwm, 0
    if turn_dir > 0:
        return pivot_pwm, -pivot_pwm, 0
    return 0.0, 0.0, 0


fd = None
old_settings = None
stop_event = threading.Event()
input_state = InputState()

try:
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    tty.setcbreak(fd)
    log.info("terminal initialized")
except Exception as exc:
    log.error("terminal init failed: %s", exc)
    raise SystemExit(1)

reader = threading.Thread(target=input_reader, args=(stop_event, input_state), daemon=True)
reader.start()

# Control state
brake_latch = False
direction_block_until = 0.0
last_drive_direction = 0

hold_direction = 0
hold_start_time = 0.0
active_level_index = 0

last_throttle_seen = 0.0
last_throttle_direction = 0
last_turn_seen = 0.0

try:
    log.info("robot3 started - W/S move, A/D turn, SPACE brake latch, Q quit")

    while True:
        now = time.time()
        last_seen, brake_pressed, quit_requested = input_state.snapshot()

        if quit_requested:
            raise KeyboardInterrupt()

        if brake_pressed:
            brake_latch = True
            hold_direction = 0
            last_drive_direction = 0
            active_level_index = 0
            hold_start_time = now
            input_state.clear_brake()
            log.info("BRAKE engaged")

        w = key_is_active(last_seen, "w", now)
        s = key_is_active(last_seen, "s", now)
        a = key_is_active(last_seen, "a", now)
        d = key_is_active(last_seen, "d", now)

        if a or d:
            last_turn_seen = now

        # Brake latch: must release all keys first.
        if brake_latch:
            apply_brake()
            if not (w or s or a or d):
                brake_latch = False
                log.info("BRAKE released; waiting for new command")
            time.sleep(control_dt)
            continue

        # Raw throttle from key activity
        if w and not s:
            throttle_raw = 1
        elif s and not w:
            throttle_raw = -1
        else:
            throttle_raw = 0

        if throttle_raw != 0:
            last_throttle_seen = now
            last_throttle_direction = throttle_raw

        # Preserve throttle intent when turning or shortly after turn release.
        if (
            throttle_raw == 0
            and last_throttle_direction != 0
            and ((a or d) or ((now - last_turn_seen) <= turn_release_grace))
        ):
            throttle_dir = last_throttle_direction
        else:
            throttle_dir = throttle_raw
            if (
                throttle_dir == 0
                and not (a or d)
                and (now - last_throttle_seen) > idle_throttle_clear
            ):
                last_throttle_direction = 0

        # Turn direction
        if a and not d:
            turn_dir = -1
        elif d and not a:
            turn_dir = 1
        else:
            turn_dir = 0

        # Speed levels
        if throttle_dir == 0:
            hold_direction = 0
            active_level_index = 0
            hold_start_time = now
            base_pwm = speed_levels[0]
        else:
            if throttle_dir != hold_direction:
                hold_direction = throttle_dir
                active_level_index = 0
                hold_start_time = now
            elapsed = max(0.0, now - hold_start_time)
            active_level_index = min(len(speed_levels) - 1, int(elapsed // level_step_seconds))
            base_pwm = speed_levels[active_level_index]

        left_cmd, right_cmd, drive_dir = resolve_motion(throttle_dir, turn_dir, base_pwm)

        # Neutral delay for direction reversal
        opposite_change = (
            drive_dir != 0
            and last_drive_direction != 0
            and drive_dir != last_drive_direction
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

        command_motor_signed(left_cmd, right_cmd)
        if drive_dir != 0:
            last_drive_direction = drive_dir
        elif abs(left_cmd) < deadband and abs(right_cmd) < deadband:
            last_drive_direction = 0

        time.sleep(control_dt)

except KeyboardInterrupt:
    pass
finally:
    stop_event.set()
    reader.join(timeout=0.5)
    stop_motors()
    if old_settings is not None and fd is not None:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    log.info("robot3 exited safely")
