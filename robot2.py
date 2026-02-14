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
# Car state
# ----------------------------
speed_A = 0.0
speed_B = 0.0
acceleration = 0.02  # speed increase per loop
deceleration = 0.01  # speed decrease per loop
max_speed = 1.0

direction_A = True  # True=forward, False=backward
direction_B = True

# If motor B is wired reversed relative to A, set True
invert_B = True

# ----------------------------
# Key handling helpers
# ----------------------------
def get_pressed_key(timeout=0.05):
    """Check if key pressed, return key or None"""
    dr,dw,de = select.select([sys.stdin], [], [], timeout)
    if dr:
        return sys.stdin.read(1).lower()
    return None

def update_motors():
    motorA_in1.on() if direction_A else motorA_in1.off()
    motorA_in2.off() if direction_A else motorA_in2.on()

    # apply inversion for motor B if wiring is flipped
    eff_dir_B = (direction_B != invert_B)
    motorB_in3.on() if eff_dir_B else motorB_in3.off()
    motorB_in4.off() if eff_dir_B else motorB_in4.on()

    motorA_pwm.value = speed_A
    motorB_pwm.value = speed_B

# ----------------------------
# Initialize terminal
# ----------------------------
fd = sys.stdin.fileno()
old_settings = termios.tcgetattr(fd)
tty.setcbreak(fd)

try:
    print("Hold WASD to drive, release to slow down, SPACE to brake, Q to quit")
    pressed_keys = set()

    while True:
        key = get_pressed_key()
        if key:
            if key == 'q':
                break
            elif key == ' ':
                speed_A = 0
                speed_B = 0
                update_motors()
                pressed_keys.clear()
                continue
            else:
                pressed_keys.add(key)
        else:
            # No key available this loop -> treat as key release
            pressed_keys.clear()

        # ----------------------------
        # Handle forward/backward
        if 'w' in pressed_keys:
            direction_A = True
            direction_B = True
            speed_A = min(max_speed, speed_A + acceleration)
            speed_B = min(max_speed, speed_B + acceleration)
        elif 's' in pressed_keys:
            direction_A = False
            direction_B = False
            speed_A = min(max_speed, speed_A + acceleration)
            speed_B = min(max_speed, speed_B + acceleration)
        else:
            # Gradual slow down
            speed_A = max(0, speed_A - deceleration)
            speed_B = max(0, speed_B - deceleration)

        # ----------------------------
        # Handle turning (continuous while held)
        if 'a' in pressed_keys:
            # Reduce left motor to turn left
            speed_A = max(0, speed_A * 0.5)
        if 'd' in pressed_keys:
            # Reduce right motor to turn right
            speed_B = max(0, speed_B * 0.5)

        update_motors()
        time.sleep(0.05)

except KeyboardInterrupt:
    pass
finally:
    speed_A = 0
    speed_B = 0
    update_motors()
    termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    print("\nExited safely")
