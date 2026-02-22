from gpiozero import PWMLED, DigitalOutputDevice
import time

# Motor pins
motorA_in1 = DigitalOutputDevice(17)  # Left IN1
motorA_in2 = DigitalOutputDevice(27)  # Left IN2
motorB_in3 = DigitalOutputDevice(22)  # Right IN3
motorB_in4 = DigitalOutputDevice(23)  # Right IN4
motorA_pwm = PWMLED(12)               # Left PWM
motorB_pwm = PWMLED(13)               # Right PWM

invert_B = True

# Fixed drive values (no input, no speed levels)
left_pwm = 0.65
right_pwm = 0.15


def set_motor_direction(left_forward, right_forward):
    motorA_in1.on() if left_forward else motorA_in1.off()
    motorA_in2.off() if left_forward else motorA_in2.on()

    effective_right_forward = (right_forward != invert_B)
    motorB_in3.on() if effective_right_forward else motorB_in3.off()
    motorB_in4.off() if effective_right_forward else motorB_in4.on()


def stop_motors():
    motorA_pwm.value = 0.0
    motorB_pwm.value = 0.0
    motorA_in1.off()
    motorA_in2.off()
    motorB_in3.off()
    motorB_in4.off()


try:
    # Forward direction on both sides.
    set_motor_direction(True, True)

    # Differential speed: right side slower -> continuous right turn.
    motorA_pwm.value = left_pwm
    motorB_pwm.value = right_pwm

    while True:
        time.sleep(0.1)
except KeyboardInterrupt:
    pass
finally:
    stop_motors()
