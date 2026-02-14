from gpiozero import PWMLED, DigitalOutputDevice
from time import sleep
import sys, tty, termios

# ----------------------------
# Motor pins
# ----------------------------
motorA_in1 = DigitalOutputDevice(17)
motorA_in2 = DigitalOutputDevice(27)
motorB_in3 = DigitalOutputDevice(22)
motorB_in4 = DigitalOutputDevice(23)

motorA_pwm = PWMLED(12)  # ENA
motorB_pwm = PWMLED(13)  # ENB

current_speed_A = 0
current_speed_B = 0

# ----------------------------
# Motor control functions
# ----------------------------
def motor_A(forward=True, speed=0.5):
    motorA_in1.on() if forward else motorA_in1.off()
    motorA_in2.off() if forward else motorA_in2.on()
    global current_speed_A
    current_speed_A = speed
    motorA_pwm.value = speed

def motor_B(forward=True, speed=0.5):
    motorB_in3.on() if forward else motorB_in3.off()
    motorB_in4.off() if forward else motorB_in4.on()
    global current_speed_B
    current_speed_B = speed
    motorB_pwm.value = speed

def smooth_brake(step=0.05, delay=0.05):
    global current_speed_A, current_speed_B
    while current_speed_A > 0 or current_speed_B > 0:
        if current_speed_A > 0:
            current_speed_A = max(0, current_speed_A - step)
            motorA_pwm.value = current_speed_A
        if current_speed_B > 0:
            current_speed_B = max(0, current_speed_B - step)
            motorB_pwm.value = current_speed_B
        sleep(delay)
    motorA_in1.off(); motorA_in2.off()
    motorB_in3.off(); motorB_in4.off()

# ----------------------------
# Keyboard input helper
# ----------------------------
def getch():
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch

# ----------------------------
# Interactive loop
# ----------------------------
try:
    print("WASD = move, SPACE = smooth brake, Q = quit")

    while True:
        key = getch().lower()
        if key == 'w':
            motor_A(True, 0.7)
            motor_B(True, 0.7)
        elif key == 's':
            motor_A(False, 0.7)
            motor_B(False, 0.7)
        elif key == 'a':
            motor_A(False, 0.5)
            motor_B(True, 0.7)
        elif key == 'd':
            motor_A(True, 0.7)
            motor_B(False, 0.5)
        elif key == ' ':
            smooth_brake()
        elif key == 'q':
            break

except KeyboardInterrupt:
    pass

finally:
    smooth_brake()
    print("\nExited safely")
