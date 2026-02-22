import time
import sys
import tty
import termios
import select
import argparse
from dataclasses import dataclass


# Key timing
KEY_TIMEOUT = 0.18
THROTTLE_HOLD_WINDOW = 0.30
TURN_THROTTLE_GRACE = 0.80
IDLE_THROTTLE_CLEAR = 1.20
LOOP_DT = 0.02

# Speed levels (normalized PWM)
SPEED_LEVELS = [0.38, 0.62, 0.92]
LEVEL_STEP_SECONDS = 2.0

# Turn behavior
TURN_YIELD_RATIO = 0.54
PIVOT_PWM = 0.68

# Safety
DIRECTION_CHANGE_DELAY = 0.20


@dataclass
class ControlSnapshot:
    state: str
    throttle_direction: int
    turn_direction: int
    speed_level: int
    base_pwm: float
    left_cmd: float
    right_cmd: float
    blocked_until: float


def get_pressed_key(timeout=0.01):
    dr, _, _ = select.select([sys.stdin], [], [], timeout)
    if dr:
        return sys.stdin.read(1).lower()
    return None


def key_active(key_state, key_last_seen, key, now, hold_window=0.0):
    if key_state[key]:
        return True
    if hold_window <= 0.0:
        return False
    return (now - key_last_seen[key]) <= hold_window


def release_timed_out_keys(key_state, key_last_seen, now):
    for key in key_state:
        if key_state[key] and (now - key_last_seen[key]) > KEY_TIMEOUT:
            key_state[key] = False


def resolve_motion_command(throttle_direction, a, d, base_pwm):
    moving_forward = throttle_direction > 0
    moving_backward = throttle_direction < 0
    turn_left = a and not d
    turn_right = d and not a

    if moving_forward or moving_backward:
        sign = 1.0 if moving_forward else -1.0
        outer = base_pwm
        inner = base_pwm * TURN_YIELD_RATIO
        if turn_left:
            left = inner
            right = outer
        elif turn_right:
            left = outer
            right = inner
        else:
            left = outer
            right = outer
        return left * sign, right * sign, (1 if moving_forward else -1), ("TURN_LEFT_MOVE" if turn_left else ("TURN_RIGHT_MOVE" if turn_right else "MOVE"))

    if turn_left:
        return -PIVOT_PWM, PIVOT_PWM, 0, "PIVOT_LEFT"
    if turn_right:
        return PIVOT_PWM, -PIVOT_PWM, 0, "PIVOT_RIGHT"
    return 0.0, 0.0, 0, "IDLE"


def main():
    parser = argparse.ArgumentParser(description="Console-only debug controller for robot3 logic")
    parser.add_argument("--verbose", action="store_true", help="Print state every loop")
    args = parser.parse_args()

    key_state = {"w": False, "s": False, "a": False, "d": False}
    key_last_seen = {"w": 0.0, "s": 0.0, "a": 0.0, "d": 0.0}

    brake_latch = False
    hold_direction = 0
    hold_start_time = 0.0
    active_level_index = 0
    last_drive_direction = 0
    last_throttle_seen = 0.0
    last_throttle_direction = 0
    direction_block_until = 0.0

    last_state = None
    last_print = 0.0

    fd = None
    old_settings = None

    try:
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        tty.setcbreak(fd)
        print("[INFO] terminal initialized")
        print("[INFO] keys: W/S move, A/D turn, SPACE brake latch, Q quit")
    except Exception as exc:
        print(f"[ERROR] terminal init failed: {exc}")
        return

    try:
        while True:
            now = time.time()

            while True:
                key = get_pressed_key(timeout=0.001)
                if not key:
                    break
                if key == "q":
                    raise KeyboardInterrupt()
                if key == " ":
                    brake_latch = True
                    hold_direction = 0
                    last_drive_direction = 0
                    hold_start_time = now
                    active_level_index = 0
                    print("[STATE] BRAKE_LATCHED")
                    continue
                if key in key_state:
                    key_state[key] = True
                    key_last_seen[key] = now

            release_timed_out_keys(key_state, key_last_seen, now)

            w = key_active(key_state, key_last_seen, "w", now, THROTTLE_HOLD_WINDOW)
            s = key_active(key_state, key_last_seen, "s", now, THROTTLE_HOLD_WINDOW)
            a = key_active(key_state, key_last_seen, "a", now, 0.0)
            d = key_active(key_state, key_last_seen, "d", now, 0.0)

            if brake_latch:
                current_state = "BRAKE_LATCHED"
                snap = ControlSnapshot(current_state, 0, 0, 1, SPEED_LEVELS[0], 0.0, 0.0, direction_block_until)
                if last_state != current_state:
                    print(f"[TRANSITION] {last_state} -> {current_state}")
                last_state = current_state
                if not (w or s or a or d):
                    brake_latch = False
                    print("[STATE] BRAKE_RELEASED (waiting new command)")
                if args.verbose and (now - last_print) >= 0.05:
                    print_snapshot(now, snap, w, s, a, d)
                    last_print = now
                time.sleep(LOOP_DT)
                continue

            if w and not s:
                throttle_raw = 1
            elif s and not w:
                throttle_raw = -1
            else:
                throttle_raw = 0

            if throttle_raw != 0:
                last_throttle_seen = now
                last_throttle_direction = throttle_raw

            if (
                throttle_raw == 0
                and (a or d)
                and last_throttle_direction != 0
                and (now - last_throttle_seen) <= TURN_THROTTLE_GRACE
            ):
                throttle_direction = last_throttle_direction
            else:
                throttle_direction = throttle_raw
                if (
                    throttle_direction == 0
                    and not (a or d)
                    and (now - last_throttle_seen) > IDLE_THROTTLE_CLEAR
                ):
                    last_throttle_direction = 0

            if throttle_direction == 0:
                hold_direction = 0
                hold_start_time = now
                active_level_index = 0
                base_pwm = SPEED_LEVELS[0]
            else:
                if throttle_direction != hold_direction:
                    hold_direction = throttle_direction
                    hold_start_time = now
                    active_level_index = 0
                elapsed = max(0.0, now - hold_start_time)
                active_level_index = min(len(SPEED_LEVELS) - 1, int(elapsed // LEVEL_STEP_SECONDS))
                base_pwm = SPEED_LEVELS[active_level_index]

            left_cmd, right_cmd, drive_direction, mode_state = resolve_motion_command(
                throttle_direction, a, d, base_pwm
            )

            opposite_change = (
                drive_direction != 0
                and last_drive_direction != 0
                and drive_direction != last_drive_direction
            )
            if opposite_change:
                direction_block_until = now + DIRECTION_CHANGE_DELAY
                last_drive_direction = 0
                mode_state = "DIR_CHANGE_BLOCK"
                left_cmd = 0.0
                right_cmd = 0.0

            if now < direction_block_until:
                mode_state = "DIR_CHANGE_BLOCK"
                left_cmd = 0.0
                right_cmd = 0.0

            if drive_direction != 0 and mode_state != "DIR_CHANGE_BLOCK":
                last_drive_direction = drive_direction
            elif mode_state == "IDLE":
                last_drive_direction = 0

            current_state = mode_state
            if last_state != current_state:
                print(f"[TRANSITION] {last_state} -> {current_state}")
                last_state = current_state

            snap = ControlSnapshot(
                state=current_state,
                throttle_direction=throttle_direction,
                turn_direction=(-1 if (a and not d) else (1 if (d and not a) else 0)),
                speed_level=active_level_index + 1,
                base_pwm=base_pwm,
                left_cmd=left_cmd,
                right_cmd=right_cmd,
                blocked_until=direction_block_until,
            )

            if args.verbose and (now - last_print) >= 0.05:
                print_snapshot(now, snap, w, s, a, d)
                last_print = now

            time.sleep(LOOP_DT)

    except KeyboardInterrupt:
        pass
    finally:
        if old_settings is not None and fd is not None:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        print("[INFO] exited safely")


def print_snapshot(now, snap, w, s, a, d):
    print(
        "[VERBOSE] "
        f"t={now:.2f} "
        f"state={snap.state} "
        f"keys(w,s,a,d)={int(w)}{int(s)}{int(a)}{int(d)} "
        f"throttle={snap.throttle_direction} "
        f"turn={snap.turn_direction} "
        f"lvl={snap.speed_level} "
        f"base={snap.base_pwm:.2f} "
        f"L={snap.left_cmd:.2f} "
        f"R={snap.right_cmd:.2f} "
        f"block_until={snap.blocked_until:.2f}"
    )


if __name__ == "__main__":
    main()
