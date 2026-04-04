"""
Motor controller abstraction layer.
Supports both real hardware control and mock (simulation) mode.
"""

import logging
from typing import Optional

logger = logging.getLogger(__name__)


class MotorController:
    """Abstract motor controller - controls left and right wheel PWM."""

    # GPIO Pin Configuration
    # Motor A (Left)
    MOTOR_A_IN1 = 17   # Direction control
    MOTOR_A_IN2 = 27   # Direction control
    MOTOR_A_PWM = 12   # Speed control (ENA)
    
    # Motor B (Right) - NOTE: REVERSED PINNING!
    MOTOR_B_IN3 = 22   # Direction control (inverted)
    MOTOR_B_IN4 = 23   # Direction control (inverted)
    MOTOR_B_PWM = 13   # Speed control (ENB)
    
    # PWM Configuration
    PWM_FREQUENCY = 1000  # Hz
    MIN_PWM = 0.0

    def __init__(self, use_mock: bool = True):
        """
        Initialize motor controller.
        
        Args:
            use_mock: If True, use mock mode (simulation). If False, use hardware mode.
        """
        self.use_mock = use_mock
        self.left_pwm = 0.0
        self.right_pwm = 0.0
        self.is_stopped = True
        
        # Hardware objects (only for non-mock mode)
        self.motor_a_in1 = None
        self.motor_a_in2 = None
        self.motor_a_pwm = None
        self.motor_b_in3 = None
        self.motor_b_in4 = None
        self.motor_b_pwm = None
        
        if use_mock:
            logger.info("Motor controller initialized in MOCK mode (simulation)")
        else:
            logger.warning("Motor controller initialized in HARDWARE mode - ensure GPIO configured!")
            self._init_hardware()
    
    def _init_hardware(self):
        """Initialize hardware GPIO and PWM using gpiozero."""
        try:
            from gpiozero import PWMLED, DigitalOutputDevice
            
            # Motor A (Left) - Standard pinning
            self.motor_a_in1 = DigitalOutputDevice(self.MOTOR_A_IN1)
            self.motor_a_in2 = DigitalOutputDevice(self.MOTOR_A_IN2)
            self.motor_a_pwm = PWMLED(self.MOTOR_A_PWM)
            
            # Motor B (Right) - Reversed pinning
            self.motor_b_in3 = DigitalOutputDevice(self.MOTOR_B_IN3)
            self.motor_b_in4 = DigitalOutputDevice(self.MOTOR_B_IN4)
            self.motor_b_pwm = PWMLED(self.MOTOR_B_PWM)
            
            logger.info(
                f"GPIO initialized: "
                f"Motor A (in1={self.MOTOR_A_IN1}, in2={self.MOTOR_A_IN2}, pwm={self.MOTOR_A_PWM}), "
                f"Motor B (in3={self.MOTOR_B_IN3}, in4={self.MOTOR_B_IN4}, pwm={self.MOTOR_B_PWM})"
            )
        except ImportError:
            logger.error("gpiozero library not available - cannot run in hardware mode")
            raise
        except Exception as e:
            logger.error(f"Failed to initialize GPIO hardware: {e}")
            raise
    
    def set_speed(self, left_pwm: float, right_pwm: float) -> None:
        """
        Set PWM speed for left and right motors.
        
        Args:
            left_pwm: Left motor PWM (0.0 = stopped, 1.0 = full speed)
            right_pwm: Right motor PWM (0.0 = stopped, 1.0 = full speed)
        
        Valid range: -1.0 to 1.0
            Negative = reverse direction
            0.0 = stop
            Positive = forward direction
        """
        # Clamp values to [-1.0, 1.0]
        left_pwm = max(-1.0, min(1.0, left_pwm))
        right_pwm = max(-1.0, min(1.0, right_pwm))
        
        self.left_pwm = left_pwm
        self.right_pwm = right_pwm
        self.is_stopped = (left_pwm == 0.0 and right_pwm == 0.0)
        
        if self.use_mock:
            # Mock mode: just store values, don't write to GPIO
            logger.debug(f"PWM set: LEFT={left_pwm:.2f}, RIGHT={right_pwm:.2f}")
        else:
            # Hardware mode: write to GPIO (stub)
            self._write_hardware(left_pwm, right_pwm)
    
    def _write_hardware(self, left_pwm: float, right_pwm: float) -> None:
        """Write PWM values to hardware GPIO."""
        try:
            # Motor A (Left) - Standard pinning
            if left_pwm >= 0:
                # Forward
                self.motor_a_in1.on()
                self.motor_a_in2.off()
            else:
                # Backward
                self.motor_a_in1.off()
                self.motor_a_in2.on()
            self.motor_a_pwm.value = abs(left_pwm)
            
            # Motor B (Right) - REVERSED PINNING
            # When right_pwm >= 0 (forward), we set in3=OFF, in4=ON (inverted!)
            # When right_pwm < 0 (backward), we set in3=ON, in4=OFF (inverted!)
            if right_pwm >= 0:
                # Forward - but pins are inverted
                self.motor_b_in3.off()
                self.motor_b_in4.on()
            else:
                # Backward - but pins are inverted
                self.motor_b_in3.on()
                self.motor_b_in4.off()
            self.motor_b_pwm.value = abs(right_pwm)
            
            logger.debug(f"GPIO write: Motor A in1={self.motor_a_in1.is_active}, in2={self.motor_a_in2.is_active}, pwm={abs(left_pwm):.2f} | "
                        f"Motor B in3={self.motor_b_in3.is_active}, in4={self.motor_b_in4.is_active}, pwm={abs(right_pwm):.2f}")
        except Exception as e:
            logger.error(f"Error writing to GPIO: {e}")
            raise
        pass
    
    def stop(self) -> None:
        """Stop both motors immediately (emergency stop)."""
        logger.info("Motor controller STOP called")
        self.set_speed(0.0, 0.0)
    
    def estop(self) -> None:
        """Emergency stop - hard stop both motors."""
        logger.warning("EMERGENCY STOP")
        self.set_speed(0.0, 0.0)
    
    def close(self) -> None:
        """Cleanup motor controller resources."""
        self.stop()
        if not self.use_mock:
            self._close_hardware()
        logger.info("Motor controller closed")
    
    def _close_hardware(self) -> None:
        """Close hardware resources and cleanup GPIO."""
        try:
            if self.motor_a_pwm:
                self.motor_a_pwm.close()
            if self.motor_b_pwm:
                self.motor_b_pwm.close()
            if self.motor_a_in1:
                self.motor_a_in1.close()
            if self.motor_a_in2:
                self.motor_a_in2.close()
            if self.motor_b_in3:
                self.motor_b_in3.close()
            if self.motor_b_in4:
                self.motor_b_in4.close()
            logger.info("GPIO resources closed")
        except Exception as e:
            logger.error(f"Error closing GPIO: {e}")
    
    def get_status(self) -> dict:
        """Get current motor status."""
        return {
            "left_pwm": self.left_pwm,
            "right_pwm": self.right_pwm,
            "is_stopped": self.is_stopped,
            "mode": "mock" if self.use_mock else "hardware"
        }
