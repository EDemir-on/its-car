"""
Motor controller abstraction layer.
Supports both real hardware control and mock (simulation) mode.
"""

import logging
from typing import Optional

logger = logging.getLogger(__name__)


class MotorController:
    """Abstract motor controller - controls left and right wheel PWM."""

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
        
        if use_mock:
            logger.info("Motor controller initialized in MOCK mode (simulation)")
        else:
            logger.warning("Motor controller initialized in HARDWARE mode - ensure GPIO configured!")
            self._init_hardware()
    
    def _init_hardware(self):
        """Initialize hardware GPIO and PWM (stub for real implementation)."""
        # TODO: Implement actual RPi GPIO setup here
        # Example: import RPi.GPIO as GPIO, setup PWM on GPIO pins
        pass
    
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
        """Write PWM values to hardware (stub implementation)."""
        # TODO: Implement actual GPIO PWM write here
        # Example: self.left_motor.ChangeDutyCycle(left_pwm * 100)
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
        """Close hardware resources (stub implementation)."""
        # TODO: Implement GPIO cleanup here
        pass
    
    def get_status(self) -> dict:
        """Get current motor status."""
        return {
            "left_pwm": self.left_pwm,
            "right_pwm": self.right_pwm,
            "is_stopped": self.is_stopped,
            "mode": "mock" if self.use_mock else "hardware"
        }
