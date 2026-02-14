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
    motorB_in3.on() if direction_B else motorB_in3.off()
    motorB_in4.off() if direction_B else motorB_in4.on()
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
                continue
            else:
                pressed_keys.add(key)

        # Remove keys that are no longer pressed
        # (simplified: we just accelerate if key pressed, otherwise decelerate)

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
        # Handle turning
        if 'a' in pressed_keys:
            # Slow left motor for left turn
            speed_A = max(0, speed_A - acceleration/2)
        if 'd' in pressed_keys:
            # Slow right motor for right turn
            speed_B = max(0, speed_B - acceleration/2)

        update_motors()
        pressed_keys.clear()  # reset each loop
        time.sleep(0.05)

except KeyboardInterrupt:
    pass
finally:
    speed_A = 0
    speed_B = 0
    update_motors()
    termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    print("\nExited safely")
