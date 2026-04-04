"""
Robot movement state machine.
Core logic for state transitions and PWM control.
"""

import logging
import time
from typing import Optional
from state_definitions import (
    MovementState,
    Direction,
    SpeedLevel,
    TurnIntensity,
    RAMP_TIME_ACCELERATE,
    RAMP_TIME_BRAKE,
    MIN_PWM,
    MIN_SPEED_FOR_MOVEMENT,
    TURN_RATIO_MOVING,
    TURN_RATIO_GENTLE,
    DIRECTION_CHANGE_NEUTRAL_TIME,
)
from motor_controller import MotorController

logger = logging.getLogger(__name__)


class RobotMovement:
    """State machine for robot movement control."""

    def __init__(self, motor_controller: Optional[MotorController] = None):
        """
        Initialize robot movement state machine.
        
        Args:
            motor_controller: MotorController instance (creates default mock if None)
        """
        self.motor = motor_controller or MotorController(use_mock=True)
        
        # State variables
        self.current_state = MovementState.STATIONARY
        self.direction = Direction.NEUTRAL
        self.current_speed_level = SpeedLevel.L0
        self.turn_intensity = 0.0  # -1.0 (sharp left) to 1.0 (sharp right)
        
        # PWM ramping
        self.target_pwm = 0.0
        self.current_pwm = 0.0
        self.last_pwm_update = time.time()
        
        # Direction change safety
        self.last_direction_change_time = time.time()
        
        # Debug tracking
        self.state_change_count = 0
        
        logger.info("RobotMovement initialized: STATIONARY, NEUTRAL, L0")

    def update(self, delta_time: float) -> None:
        """
        Main update loop - call this regularly (every ~100ms).
        Handles PWM ramping and state transitions.
        
        Args:
            delta_time: Time elapsed since last update (seconds)
        """
        # Ramp PWM toward target
        self._ramp_pwm(delta_time)
        
        # Compute and apply motor speeds
        left_pwm, right_pwm = self._compute_pwm_values()
        self.motor.set_speed(left_pwm, right_pwm)

    def _ramp_pwm(self, delta_time: float) -> None:
        """
        Smoothly ramp current PWM toward target PWM.
        
        Args:
            delta_time: Time elapsed since last update (seconds)
        """
        if self.current_pwm == self.target_pwm:
            return
        
        if self.current_state == MovementState.BRAKING:
            ramp_rate = 1.0 / RAMP_TIME_BRAKE  # Full speed change per second
        else:
            ramp_rate = 1.0 / RAMP_TIME_ACCELERATE

        if self.current_pwm < self.target_pwm:
            self.current_pwm = min(self.target_pwm, self.current_pwm + ramp_rate * delta_time)
        else:
            self.current_pwm = max(self.target_pwm, self.current_pwm - ramp_rate * delta_time)
        
        # Clamp to 0 if very close
        if abs(self.current_pwm) < 0.01:
            self.current_pwm = 0.0

    def _compute_pwm_values(self) -> tuple[float, float]:
        """
        Compute left and right PWM values based on current state, direction, and turn.
        
        Returns:
            (left_pwm, right_pwm): PWM values in range [-1.0, 1.0]
        """
        # Base PWM from current speed
        base_pwm = self.current_pwm
        
        # Apply direction
        if self.direction == Direction.REVERSE:
            base_pwm = -base_pwm
        elif self.direction == Direction.NEUTRAL:
            base_pwm = 0.0
        
        # Apply turning
        left_pwm, right_pwm = self._apply_turning_ratio(base_pwm)
        
        return left_pwm, right_pwm

    def _apply_turning_ratio(self, base_pwm: float) -> tuple[float, float]:
        """
        Apply turning behavior to PWM values.
        Different logic for pivot (stationary) vs. leaning (moving).
        
        Args:
            base_pwm: Base forward/reverse PWM before turning
            
        Returns:
            (left_pwm, right_pwm): PWM with turning applied
        """
        if self.turn_intensity == 0.0:
            # Straight - equal PWM
            return base_pwm, base_pwm
        
        if self.current_state == MovementState.STATIONARY_TURN:
            # Pivot turn: opposite wheels reverse direction
            return self._pivot_turn(base_pwm)
        elif self.current_state == MovementState.MOVING_TURN:
            # Lean turn: reduce inner wheel speed
            return self._lean_turn(base_pwm)
        else:
            # No turning in other states
            return base_pwm, base_pwm

    def _pivot_turn(self, base_pwm: float) -> tuple[float, float]:
        """
        Stationary pivot turn: spin in place.
        Outer wheel forward, inner wheel reverse.
        
        Args:
            base_pwm: Ignored (should be 0 in STATIONARY_TURN)
            
        Returns:
            (left_pwm, right_pwm): Opposite wheel directions for pivot
        """
        # Turn intensity: positive = turn right (right wheel forward, left wheel reverse)
        #               negative = turn left (left wheel forward, right wheel reverse)
        
        turn_speed = abs(self.turn_intensity)  # 0.0 to 1.0
        
        if self.turn_intensity > 0:
            # Turn right: right wheel forward, left wheel reverse
            left_pwm = -turn_speed
            right_pwm = turn_speed
        else:
            # Turn left: left wheel forward, right wheel reverse
            left_pwm = turn_speed
            right_pwm = -turn_speed
        
        return left_pwm, right_pwm

    def _lean_turn(self, base_pwm: float) -> tuple[float, float]:
        """
        Moving lean turn: reduce inner wheel speed while moving.
        Outer wheel at full base_pwm, inner wheel reduced.
        
        Args:
            base_pwm: Base forward/reverse PWM
            
        Returns:
            (left_pwm, right_pwm): Reduced inner wheel for leaning turn
        """
        # Determine turn ratio based on intensity magnitude
        abs_intensity = abs(self.turn_intensity)
        if abs_intensity < 0.3:
            # Gentle turn
            ratio = TURN_RATIO_GENTLE
        else:
            # Medium/sharp turn
            ratio = TURN_RATIO_MOVING
        
        if self.turn_intensity > 0:
            # Turn right: reduce left wheel speed
            left_pwm = base_pwm * ratio
            right_pwm = base_pwm
        else:
            # Turn left: reduce right wheel speed
            left_pwm = base_pwm
            right_pwm = base_pwm * ratio
        
        return left_pwm, right_pwm

    # ========== State Transition Methods ==========

    def accelerate(self) -> bool:
        """
        Accelerate in current direction (or start forward if stationary).
        Increases speed level toward maximum.
        
        Returns:
            True if transition successful, False otherwise
        """
        # Can only accelerate from STATIONARY or ACCELERATING or COASTING
        if self.current_state not in [MovementState.STATIONARY, MovementState.ACCELERATING, MovementState.COASTING]:
            logger.warning(f"Cannot accelerate from {self.current_state}")
            return False
        
        # Check direction safety
        if self.direction == Direction.REVERSE:
            if not self._check_direction_safe():
                logger.warning("Direction change blocked by safety timer")
                return False
            self.direction = Direction.FORWARD
            self.last_direction_change_time = time.time()
        else:
            self.direction = Direction.FORWARD
        
        # Increase speed level if not at max
        if self.current_speed_level != SpeedLevel.L3:
            old_level = self.current_speed_level
            # Advance to next level
            level_order = [SpeedLevel.L0, SpeedLevel.L1, SpeedLevel.L2, SpeedLevel.L3]
            current_idx = level_order.index(self.current_speed_level)
            self.current_speed_level = level_order[min(current_idx + 1, 3)]
            logger.info(f"Speed: {old_level.name} → {self.current_speed_level.name}")
        
        # Update state
        old_state = self.current_state
        if self.current_speed_level == SpeedLevel.L0:
            self.current_state = MovementState.STATIONARY
        else:
            self.current_state = MovementState.ACCELERATING
        
        self.target_pwm = self.current_speed_level.value
        self._log_state_change(old_state)
        return True

    def coast(self) -> bool:
        """
        Coast - stop accelerating, maintain current speed.
        
        Returns:
            True if transition successful, False otherwise
        """
        if self.current_state not in [MovementState.ACCELERATING, MovementState.MOVING_TURN]:
            logger.warning(f"Cannot coast from {self.current_state}")
            return False
        
        old_state = self.current_state
        self.current_state = MovementState.COASTING
        self.target_pwm = self.current_speed_level.value
        self._log_state_change(old_state)
        return True

    def brake(self) -> bool:
        """
        Brake - active deceleration to stop.
        
        Returns:
            True if transition successful, False otherwise
        """
        if self.current_state == MovementState.BRAKING or self.current_state == MovementState.STATIONARY:
            return False
        
        old_state = self.current_state
        self.current_state = MovementState.BRAKING
        self.target_pwm = 0.0
        self._log_state_change(old_state)
        return True

    def stop(self) -> bool:
        """
        Full stop - immediately go to STATIONARY.
        Used for emergency stop or normal shutdown.
        
        Returns:
            True if successful
        """
        old_state = self.current_state
        self.current_state = MovementState.STATIONARY
        self.direction = Direction.NEUTRAL
        self.current_speed_level = SpeedLevel.L0
        self.turn_intensity = 0.0
        self.target_pwm = 0.0
        self.current_pwm = 0.0
        
        self.motor.stop()
        self._log_state_change(old_state)
        return True

    def turn_left(self, intensity: float = 0.6) -> bool:
        """
        Start turning left.
        In STATIONARY state: pivot turn.
        In COASTING/ACCELERATING state: lean turn.
        
        Args:
            intensity: Turn intensity (0.0 to 1.0, where 1.0 is sharp pivot)
            
        Returns:
            True if successful
        """
        intensity = max(-1.0, min(0.0, intensity))  # Clamp to [-1.0, 0.0] for left
        self.turn_intensity = intensity
        
        if self.current_state == MovementState.STATIONARY:
            old_state = self.current_state
            self.current_state = MovementState.STATIONARY_TURN
            self._log_state_change(old_state)
            logger.info(f"Turn LEFT (pivot): intensity={intensity:.2f}")
            return True
        elif self.current_state in [MovementState.COASTING, MovementState.ACCELERATING]:
            old_state = self.current_state
            self.current_state = MovementState.MOVING_TURN
            self._log_state_change(old_state)
            logger.info(f"Turn LEFT (leaning): intensity={intensity:.2f}")
            return True
        else:
            logger.warning(f"Cannot turn from {self.current_state}")
            return False

    def turn_right(self, intensity: float = 0.6) -> bool:
        """
        Start turning right.
        In STATIONARY state: pivot turn.
        In COASTING/ACCELERATING state: lean turn.
        
        Args:
            intensity: Turn intensity (0.0 to 1.0, where 1.0 is sharp pivot)
            
        Returns:
            True if successful
        """
        intensity = max(0.0, min(1.0, intensity))  # Clamp to [0.0, 1.0] for right
        self.turn_intensity = intensity
        
        if self.current_state == MovementState.STATIONARY:
            old_state = self.current_state
            self.current_state = MovementState.STATIONARY_TURN
            self._log_state_change(old_state)
            logger.info(f"Turn RIGHT (pivot): intensity={intensity:.2f}")
            return True
        elif self.current_state in [MovementState.COASTING, MovementState.ACCELERATING]:
            old_state = self.current_state
            self.current_state = MovementState.MOVING_TURN
            self._log_state_change(old_state)
            logger.info(f"Turn RIGHT (leaning): intensity={intensity:.2f}")
            return True
        else:
            logger.warning(f"Cannot turn from {self.current_state}")
            return False

    def straighten(self) -> bool:
        """
        Stop turning - go straight or return to previous state.
        
        Returns:
            True if successful
        """
        if self.current_state == MovementState.STATIONARY_TURN:
            old_state = self.current_state
            self.current_state = MovementState.STATIONARY
            self.turn_intensity = 0.0
            self._log_state_change(old_state)
            return True
        elif self.current_state == MovementState.MOVING_TURN:
            old_state = self.current_state
            self.current_state = MovementState.COASTING
            self.turn_intensity = 0.0
            self._log_state_change(old_state)
            return True
        else:
            logger.warning(f"Cannot straighten from {self.current_state}")
            return False

    def reverse(self, intensity: float = 0.4) -> bool:
        """
        Start reversing (backward).
        Must be in STATIONARY state for safety.
        
        Args:
            intensity: Reverse speed level (0.0 to 1.0)
            
        Returns:
            True if successful
        """
        if self.current_state != MovementState.STATIONARY:
            logger.warning(f"Cannot reverse from {self.current_state} - must be STATIONARY")
            return False
        
        if not self._check_direction_safe():
            logger.warning("Direction change blocked by safety timer")
            return False
        
        old_state = self.current_state
        self.direction = Direction.REVERSE
        self.current_speed_level = SpeedLevel.L1  # Start at L1 for reverse
        self.current_state = MovementState.ACCELERATING
        self.target_pwm = self.current_speed_level.value
        self.last_direction_change_time = time.time()
        
        self._log_state_change(old_state)
        logger.info(f"REVERSE: L1, intensity={intensity:.2f}")
        return True

    # ========== Safety & Helper Methods ==========

    def _check_direction_safe(self) -> bool:
        """Check if enough time has passed to safely change direction."""
        elapsed = time.time() - self.last_direction_change_time
        if elapsed < DIRECTION_CHANGE_NEUTRAL_TIME:
            return False
        return True

    def _log_state_change(self, old_state: MovementState) -> None:
        """Log state transition with details."""
        self.state_change_count += 1
        logger.info(
            f"[{self.state_change_count}] {old_state.name:15} → {self.current_state.name:15} | "
            f"Direction: {self.direction.name:7} | Speed: {self.current_speed_level.name} | "
            f"Turn: {self.turn_intensity:+.2f} | PWM: {self.current_pwm:.2f}→{self.target_pwm:.2f}"
        )

    def get_status(self) -> dict:
        """Get current robot status."""
        return {
            "state": self.current_state.name,
            "direction": self.direction.name,
            "speed_level": self.current_speed_level.name,
            "speed_pwm": self.current_speed_level.value,
            "turn_intensity": self.turn_intensity,
            "current_pwm": self.current_pwm,
            "target_pwm": self.target_pwm,
            "motor_status": self.motor.get_status(),
        }
