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

# If motor B is wired reversed relative to A, set True
invert_B = True

# brake state
brake_active = False

# key timing to detect release (allow OS key-repeat gaps)
last_key_time = 0.0
release_delay = 0.18  # seconds

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
    # hard brake: set PWM to 0 and set both inputs same to actively brake motors
    motorA_pwm.value = 0
    motorB_pwm.value = 0
    # short the motor terminals (both inputs high) - effective brake on many drivers
    motorA_in1.on()
    motorA_in2.on()
    motorB_in3.on()
    motorB_in4.on()

def update_motors_from_state():
    """Set GPIO outputs according to base_direction, base_speed and turning accumulators.
       This function respects invert_B."""
    global base_speed, base_direction, turn_left, turn_right

    # If braking is active, leave pins in braking state elsewhere
    if brake_active:
        apply_brake()
        return

    # Calculate effective turning strength (-1..1): positive => left turn
    turn_strength = turn_left - turn_right
    if turn_strength > 0.0:
        ts = min(turn_strength, turn_max)
    elif turn_strength < 0.0:
        ts = max(turn_strength, -turn_max)
    else:
        ts = 0.0

    # If stopped and turning requested -> spin in place
    if base_speed <= 0.0001 and abs(ts) > 0.01:
        spin_speed = turn_in_place_speed * abs(ts)
        if ts > 0:  # pivot left: left backward, right forward
            dirA = False
            dirB = True
        else:       # pivot right
            dirA = True
            dirB = False
        # apply directions (with inversion for B)
        motorA_in1.on() if dirA else motorA_in1.off()
        motorA_in2.off() if dirA else motorA_in2.on()
        eff_dir_B = (dirB != invert_B)
        motorB_in3.on() if eff_dir_B else motorB_in3.off()
        motorB_in4.off() if eff_dir_B else motorB_in4.on()
        motorA_pwm.value = spin_speed
        motorB_pwm.value = spin_speed
        return

    # Moving or coasting: apply differential by reducing the inner wheel
    # inner wheel reduction = abs(ts); outer wheel remains at base_speed
    left_multiplier = 1.0
    right_multiplier = 1.0
    if ts > 0:  # left turn -> reduce left wheel
        left_multiplier = max(0.0, 1.0 - ts)
    elif ts < 0:  # right turn -> reduce right wheel
        right_multiplier = max(0.0, 1.0 - abs(ts))

    # Compute per-motor speeds and directions
    speedA = base_speed * left_multiplier
    speedB = base_speed * right_multiplier

    # Apply directions: both same as base_direction when moving
    dirA = True if base_direction >= 0 else False
    dirB = True if base_direction >= 0 else False

    # If base_direction == 0, ensure motors are stopped (handled earlier)
    # Write outputs with consideration for invert_B
    motorA_in1.on() if dirA else motorA_in1.off()
    motorA_in2.off() if dirA else motorA_in2.on()

    eff_dir_B = (dirB != invert_B)
    motorB_in3.on() if eff_dir_B else motorB_in3.off()
    motorB_in4.off() if eff_dir_B else motorB_in4.on()

    motorA_pwm.value = speedA
    motorB_pwm.value = speedB

# ----------------------------
# Initialize terminal
# ----------------------------
fd = sys.stdin.fileno()
old_settings = termios.tcgetattr(fd)
tty.setcbreak(fd)

try:
    print("Hold WASD to drive, release to slow down, SPACE to brake (holds), Q to quit")
    pressed_keys = set()

    while True:
        now = time.time()
        key = get_pressed_key(timeout=0.05)
        if key:
            last_key_time = now
            # handle quit immediately
            if key == 'q':
                break
            # brake overrides everything and is applied immediately and held
            if key == ' ':
                brake_active = True
                # zero base speed immediately
                base_speed = 0.0
                base_direction = 0
                # reset turn accumulators optionally keep them so user can resume turning
                apply_brake()
                pressed_keys.clear()
                # continue loop so brake is applied now
                continue
            # other keys add to pressed set
            pressed_keys.add(key)
        else:
            # If no input for a short time assume release of held keys
            if now - last_key_time > release_delay:
                pressed_keys.clear()

        # If brake is active, wait until a movement key is pressed to release braking
        if brake_active:
            # if user presses movement or turn keys, release brake and resume control
            if any(k in pressed_keys for k in ('w','s','a','d')):
                brake_active = False
                # restore motor pins according to new state below
                # fall through to rest of loop
            else:
                apply_brake()
                time.sleep(0.02)
                continue

        # ----------------------------
        # Handle forward/backward (W/S)
        # start slowly and keep speeding up while key held; decelerate on release
        if 'w' in pressed_keys:
            # forward
            if base_speed <= 0.0001:
                base_speed = min_torque
                startup_counter = startup_boost_steps
            elif startup_counter > 0:
                base_speed = min(max_speed, base_speed + startup_boost_increment)
                startup_counter -= 1
            else:
                base_speed = min(max_speed, base_speed + acceleration)
            base_direction = 1
        elif 's' in pressed_keys:
            # backward
            if base_speed <= 0.0001:
                base_speed = min_torque
                startup_counter = startup_boost_steps
            elif startup_counter > 0:
                base_speed = min(max_speed, base_speed + startup_boost_increment)
                startup_counter -= 1
            else:
                base_speed = min(max_speed, base_speed + acceleration)
            base_direction = -1
        else:
            # no forward/back key: slow down
            startup_counter = 0
            base_speed = max(0.0, base_speed - deceleration)
            if base_speed <= 0.0001:
                base_speed = 0.0
                base_direction = 0

        # ----------------------------
        # Handle turning accumulators (A/D)
        if 'a' in pressed_keys and 'd' not in pressed_keys:
            turn_left = min(turn_max, turn_left + turn_increment)
        else:
            turn_left = max(0.0, turn_left - turn_decay)

        if 'd' in pressed_keys and 'a' not in pressed_keys:
            turn_right = min(turn_max, turn_right + turn_increment)
        else:
            turn_right = max(0.0, turn_right - turn_decay)

        # small deadzone
        if turn_left < 0.01:
            turn_left = 0.0
        if turn_right < 0.01:
            turn_right = 0.0

        # Update GPIOs from computed state
        update_motors_from_state()
        time.sleep(0.03)

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
