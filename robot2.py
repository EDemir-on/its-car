from gpiozero import PWMLED, DigitalOutputDevice
import time, sys, tty, termios, select

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
# Car state (refactored)
# ----------------------------
base_speed = 0.0         # common target speed (0..max_speed)
base_direction = 0       # 1 = forward, -1 = backward, 0 = stopped

# initial torque to overcome stiction
min_torque = 0.35

# acceleration / deceleration parameters
acceleration = 0.06      # faster ramp while key held
deceleration = 0.03      # decelerate when no key
max_speed = 1.0

# startup boost (short)
startup_boost_steps = 3
startup_boost_increment = 0.08
startup_counter = 0

# turning accumulators (independent, grow while held, decay when released)
turn_left = 0.0
turn_right = 0.0
turn_increment = 0.09
turn_decay = 0.06
turn_max = 0.95

# in-place spin speed multiplier when stopped and turning
turn_in_place_speed = 0.7 * max_speed

# scale applied to turning when the car is moving (0..1). 1 -> full turn, 0.5 -> half effect
moving_turn_scale = 0.9

# If motor B is wired reversed relative to A, set True
invert_B = True

# brake state
brake_active = False

# key timing to detect release (allow OS key-repeat gaps)
last_key_time = 0.0
release_delay = 0.25  # slightly increased
key_check_interval = 0.03
eps = 1e-4

# track last-seen timestamps per key instead of clearing the whole set
last_seen = {}  # key -> timestamp

def is_pressed(k, now):
    return (k in last_seen) and (now - last_seen[k] <= release_delay)

# ----------------------------
# Key handling helpers
# ----------------------------
def get_pressed_key(timeout=0.05):
    """Check if key pressed, return key or None"""
    dr,dw,de = select.select([sys.stdin], [], [], timeout)
    if dr:
        return sys.stdin.read(1).lower()
    return None

def apply_brake():
    """Apply fast braking (short both motor inputs to stop quickly) and zero PWM."""
    motorA_pwm.value = 0
    motorB_pwm.value = 0
    # short the motor terminals (both inputs high) - effective brake on many drivers
    motorA_in1.on()
    motorA_in2.on()
    motorB_in3.on()
    motorB_in4.on()

def update_motors_from_state():
    """Set GPIO outputs according to base_direction, base_speed and turning accumulators.
       This function respects invert_B and uses current state to decide pivot vs moving."""
    global base_speed, base_direction, turn_left, turn_right, state

    if state == STATE_BRAKE or brake_active:
        apply_brake()
        return

    # Calculate effective turning strength (-1..1): positive => left turn
    turn_strength = turn_left - turn_right
    ts = max(-turn_max, min(turn_max, turn_strength))

    # Pivot when in-place turning state or when speed ~ 0 and significant turn requested
    if (state == STATE_TURN_IN_PLACE) or (base_speed <= eps and abs(ts) > 0.01):
        spin_speed = turn_in_place_speed * abs(ts)
        if ts > 0:  # pivot left: left backward, right forward
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
        return

    # Moving or coasting: apply differential smoothly by reducing inner wheel and
    # slightly boosting outer wheel so the car keeps forward momentum while turning.
    scale = moving_turn_scale if base_speed > eps else 1.0

    left_speed = base_speed
    right_speed = base_speed

    if ts > 0.01:
        reduction = ts * scale
        left_speed = max(0.0, base_speed * (1.0 - reduction))
        right_speed = min(max_speed, base_speed * (1.0 + reduction * 0.25))
    elif ts < -0.01:
        reduction = abs(ts) * scale
        right_speed = max(0.0, base_speed * (1.0 - reduction))
        left_speed = min(max_speed, base_speed * (1.0 + reduction * 0.25))

    # Apply directions: both same as base_direction when moving
    dirA = True if base_direction >= 0 else False
    dirB = True if base_direction >= 0 else False

    motorA_in1.on() if dirA else motorA_in1.off()
    motorA_in2.off() if dirA else motorA_in2.on()

    eff_dir_B = (dirB != invert_B)
    motorB_in3.on() if eff_dir_B else motorB_in3.off()
    motorB_in4.off() if eff_dir_B else motorB_in4.on()

    motorA_pwm.value = left_speed
    motorB_pwm.value = right_speed

# ----------------------------
# Initialize terminal
# ----------------------------
fd = sys.stdin.fileno()
old_settings = termios.tcgetattr(fd)
tty.setcbreak(fd)

try:
    print("States: 1=standing 2=moving 3=decelerating 4=turn_in_place 5=turn_while_moving 6=brake")
    print("Hold WASD to drive, SPACE to brake (holds), Q to quit")
    pressed_keys = set()

    while True:
        now = time.time()
        key = get_pressed_key(timeout=0.03)
        if key:
            last_key_time = now
            last_seen[key] = now              # record last-seen timestamp
            if key == 'q':
                break
            if key == ' ':
                # immediate hard brake, overrides everything
                brake_active = True
                state = STATE_BRAKE
                base_speed = 0.0
                base_direction = 0
                turn_left = 0.0
                turn_right = 0.0
                apply_brake()
                # clear timestamps so no stale presses remain
                last_seen.clear()
                continue
        else:
            # don't clear last_seen here; timestamps expire naturally via is_pressed()
            pass

        # check keys using timestamps
        w = is_pressed('w', now)
        s = is_pressed('s', now)
        a = is_pressed('a', now)
        d = is_pressed('d', now)

        # release brake if user requests movement
        if brake_active:
            if any((w, s, a, d)):
                brake_active = False
                state = STATE_STANDING
            else:
                apply_brake()
                time.sleep(key_check_interval)
                continue

        # STATE DECISION
        # Priority: turning in place if standing and A/D pressed
        if (not w and not s) and (a or d) and base_speed <= eps:
            state = STATE_TURN_IN_PLACE
        # If moving or decelerating and A/D pressed -> turning while moving
        elif (a or d) and base_speed > eps:
            state = STATE_TURN_WHILE_MOVING
        # Movement keys without turning -> moving
        elif (w or s) and not (a or d):
            state = STATE_MOVING
        # No movement keys -> if moving, decelerate, else standing
        elif not (w or s or a or d):
            if base_speed > eps:
                state = STATE_DECELERATING
            else:
                state = STATE_STANDING
        else:
            # fallback keep previous state
            pass

        # STATE BEHAVIOR
        if state == STATE_MOVING:
            # W / S accelerate while held, no turning active
            if w:
                # forward
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
                # backward
                if base_speed <= eps:
                    base_speed = min_torque
                    startup_counter = startup_boost_steps
                elif startup_counter > 0:
                    base_speed = min(max_speed, base_speed + startup_boost_increment)
                    startup_counter -= 1
                else:
                    base_speed = min(max_speed, base_speed + acceleration)
                base_direction = -1

            # decay any small turning accumulators to balance wheels
            if not a:
                turn_left = max(0.0, turn_left - turn_decay)
            if not d:
                turn_right = max(0.0, turn_right - turn_decay)

        elif state == STATE_TURN_WHILE_MOVING:
            # keep moving forward/back while building turn accumulators
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

            # accumulate turning strength while held
            if a and not d:
                turn_left = min(turn_max, turn_left + turn_increment)
            else:
                turn_left = max(0.0, turn_left - turn_decay)
            if d and not a:
                turn_right = min(turn_max, turn_right + turn_increment)
            else:
                turn_right = max(0.0, turn_right - turn_decay)

        elif state == STATE_DECELERATING:
            # slow down gracefully and allow small turning inputs to nudge
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
            # ensure base speed is zero and build turn accumulator
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

            # if both released, decay to standing
            if turn_left <= 0.0 and turn_right <= 0.0:
                state = STATE_STANDING

        elif state == STATE_STANDING:
            # no motion; decay turning to zero
            base_speed = 0.0
            base_direction = 0
            turn_left = max(0.0, turn_left - turn_decay)
            turn_right = max(0.0, turn_right - turn_decay)
            if turn_left <= 0.0 and turn_right <= 0.0:
                turn_left = 0.0
                turn_right = 0.0

        elif state == STATE_BRAKE:
            apply_brake()
            time.sleep(key_check_interval)
            continue

        # small deadzone cleanup
        if turn_left < 0.01:
            turn_left = 0.0
        if turn_right < 0.01:
            turn_right = 0.0

        # Update GPIOs from computed state
        update_motors_from_state()
        time.sleep(key_check_interval)

except KeyboardInterrupt:
    pass
finally:
    # ensure motors stopped
    brake_active = False
    base_speed = 0.0
    base_direction = 0
    motorA_pwm.value = 0
    motorB_pwm.value = 0
    motorA_in1.off()
    motorA_in2.off()
    motorB_in3.off()
    motorB_in4.off()
    termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    print("\nExited safely")
