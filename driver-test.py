from gpiozero import PWMLED, DigitalOutputDevice
from time import sleep
#import sys, tty, termios

motorA_in1 = DigitalOutputDevice(17)
motorA_in2 = DigitalOutputDevice(27)
motorB_in3 = DigitalOutputDevice(22)
motorB_in4 = DigitalOutputDevice(23)

motorA_pwm = PWMLED(12)  # ENA
motorB_pwm = PWMLED(13)  # ENB

motorA_in1.on()
motorA_in2.off()
motorA_pwm.value=1.0
sleep(10)

motorA_in1.off()
motorA_in2.off()
motorA_pwm.value=1.0
sleep(5)

motorA_pwm.value = 0
motorB_pwm.value = 0
motorA_in1.off()
motorA_in2.off()
motorB_in3.off()
motorB_in4.off()
