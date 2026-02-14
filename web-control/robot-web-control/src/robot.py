from gpiozero import PWMLED, DigitalOutputDevice

# ----------------------------
# Motor pins
# ----------------------------
motorA_in1 = DigitalOutputDevice(17)
motorA_in2 = DigitalOutputDevice(27)
motorB_in3 = DigitalOutputDevice(22)
motorB_in4 = DigitalOutputDevice(23)

motorA_pwm = PWMLED(12)  # ENA
motorB_pwm = PWMLED(13)  # ENB

def move_forward(speed):
    motorA_in1.on()
    motorA_in2.off()
    motorB_in3.on()
    motorB_in4.off()
    motorA_pwm.value = speed
    motorB_pwm.value = speed

def move_backward(speed):
    motorA_in1.off()
    motorA_in2.on()
    motorB_in3.off()
    motorB_in4.on()
    motorA_pwm.value = speed
    motorB_pwm.value = speed

def turn_left(speed):
    motorA_in1.off()
    motorA_in2.on()
    motorB_in3.on()
    motorB_in4.off()
    motorA_pwm.value = speed
    motorB_pwm.value = speed

def turn_right(speed):
    motorA_in1.on()
    motorA_in2.off()
    motorB_in3.off()
    motorB_in4.on()
    motorA_pwm.value = speed
    motorB_pwm.value = speed

def stop():
    motorA_pwm.value = 0
    motorB_pwm.value = 0
    motorA_in1.off()
    motorA_in2.off()
    motorB_in3.off()
    motorB_in4.off()