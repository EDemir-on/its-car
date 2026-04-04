"""
Unit tests for robot movement state machine.
"""

import pytest
import time
from state_definitions import (
    MovementState,
    Direction,
    SpeedLevel,
)
from motor_controller import MotorController
from robot_movement import RobotMovement


@pytest.fixture
def robot():
    """Create a robot with mock motor controller for testing."""
    motor = MotorController(use_mock=True)
    return RobotMovement(motor_controller=motor)


@pytest.fixture
def robot_no_cleanup(robot):
    """Robot fixture without cleanup."""
    return robot


class TestInitialization:
    """Test robot initialization."""

    def test_initial_state(self, robot):
        """Robot should start in STATIONARY state."""
        assert robot.current_state == MovementState.STATIONARY
        assert robot.direction == Direction.NEUTRAL
        assert robot.current_speed_level == SpeedLevel.L0

    def test_initial_pwm(self, robot):
        """Robot should start with 0 PWM."""
        assert robot.current_pwm == 0.0
        assert robot.target_pwm == 0.0


class TestStateTransitions:
    """Test state transition logic."""

    def test_accelerate_from_stationary(self, robot):
        """Should accelerate from STATIONARY to ACCELERATING."""
        result = robot.accelerate()
        assert result is True
        assert robot.current_state == MovementState.ACCELERATING
        assert robot.direction == Direction.FORWARD
        assert robot.current_speed_level == SpeedLevel.L1

    def test_accelerate_increases_speed_level(self, robot):
        """Repeated accelerate should increase speed level."""
        robot.accelerate()
        assert robot.current_speed_level == SpeedLevel.L1
        robot.accelerate()
        assert robot.current_speed_level == SpeedLevel.L2
        robot.accelerate()
        assert robot.current_speed_level == SpeedLevel.L3
        # Should not go higher
        robot.accelerate()
        assert robot.current_speed_level == SpeedLevel.L3

    def test_coast_from_accelerating(self, robot):
        """Should transition from ACCELERATING to COASTING."""
        robot.accelerate()
        result = robot.coast()
        assert result is True
        assert robot.current_state == MovementState.COASTING

    def test_brake_from_coasting(self, robot):
        """Should transition from COASTING to BRAKING."""
        robot.accelerate()
        robot.coast()
        result = robot.brake()
        assert result is True
        assert robot.current_state == MovementState.BRAKING

    def test_stop_returns_to_stationary(self, robot):
        """Stop should return to STATIONARY from any state."""
        robot.accelerate()
        assert robot.current_state == MovementState.ACCELERATING
        
        result = robot.stop()
        assert result is True
        assert robot.current_state == MovementState.STATIONARY
        assert robot.direction == Direction.NEUTRAL
        assert robot.current_speed_level == SpeedLevel.L0

    def test_invalid_transitions(self, robot):
        """Invalid transitions should return False."""
        # Cannot coast from STATIONARY
        result = robot.coast()
        assert result is False
        
        # Cannot brake from STATIONARY
        result = robot.brake()
        assert result is False


class TestTurning:
    """Test turning mechanics."""

    def test_pivot_turn_from_stationary(self, robot):
        """Should pivot turn from STATIONARY."""
        result = robot.turn_left(intensity=-0.6)
        assert result is True
        assert robot.current_state == MovementState.STATIONARY_TURN
        assert robot.turn_intensity == -0.6

    def test_lean_turn_from_moving(self, robot):
        """Should lean turn from COASTING/ACCELERATING."""
        robot.accelerate()
        result = robot.turn_right(intensity=0.6)
        assert result is True
        assert robot.current_state == MovementState.MOVING_TURN
        assert robot.turn_intensity == 0.6

    def test_straighten_from_pivot(self, robot):
        """Should straighten from STATIONARY_TURN."""
        robot.turn_left()
        result = robot.straighten()
        assert result is True
        assert robot.current_state == MovementState.STATIONARY
        assert robot.turn_intensity == 0.0

    def test_straighten_from_moving_turn(self, robot):
        """Should straighten from MOVING_TURN to COASTING."""
        robot.accelerate()
        robot.turn_right()
        result = robot.straighten()
        assert result is True
        assert robot.current_state == MovementState.COASTING
        assert robot.turn_intensity == 0.0


class TestPWMRamping:
    """Test PWM ramping mechanics."""

    def test_pwm_ramps_up_on_accelerate(self, robot):
        """PWM should ramp up smoothly when accelerating."""
        robot.accelerate()  # target = 0.4
        initial_pwm = robot.current_pwm
        
        # Update for 0.1 second
        robot.update(0.1)
        updated_pwm = robot.current_pwm
        
        # Should have increased
        assert updated_pwm > initial_pwm
        # Should not exceed target
        assert updated_pwm <= SpeedLevel.L1.value

    def test_pwm_reaches_target(self, robot):
        """PWM should reach target within ramp time."""
        robot.accelerate()
        target = SpeedLevel.L1.value
        
        # Update for full ramp time
        robot.update(0.5)
        
        # Should be at or very close to target
        assert abs(robot.current_pwm - target) < 0.05

    def test_pwm_ramps_down_on_brake(self, robot):
        """PWM should ramp down quickly when braking."""
        robot.accelerate()
        robot.update(0.5)  # Let it ramp up
        
        robot.brake()
        current = robot.current_pwm
        robot.update(0.1)  # Brief update
        
        # Should have decreased
        assert robot.current_pwm < current


class TestDirectionSafety:
    """Test direction change safety."""

    def test_cannot_reverse_immediately_after_forward(self, robot):
        """Should not allow immediate direction reversal."""
        robot.accelerate()
        robot.stop()
        
        # Try to reverse immediately
        result = robot.reverse()
        assert result is False

    def test_can_reverse_after_safety_delay(self, robot):
        """Should allow reverse after safety delay."""
        robot.accelerate()
        robot.stop()
        
        # Wait for safety delay
        time.sleep(0.21)  # Slightly more than 0.2s
        
        result = robot.reverse()
        assert result is True
        assert robot.direction == Direction.REVERSE

    def test_cannot_change_direction_without_stop(self, robot):
        """Cannot switch direction while moving."""
        robot.accelerate()
        
        result = robot.reverse()
        assert result is False


class TestTurningRatios:
    """Test turning ratio calculations."""

    def test_pivot_turn_left(self, robot):
        """Pivot turn left should have opposite wheel directions."""
        robot.turn_left(intensity=-1.0)
        robot.update(0.01)
        
        # Check motor values
        status = robot.get_status()
        assert status['motor_status']['left_pwm'] > 0  # Left wheel forward
        assert status['motor_status']['right_pwm'] < 0  # Right wheel reverse

    def test_pivot_turn_right(self, robot):
        """Pivot turn right should have opposite wheel directions."""
        robot.turn_right(intensity=1.0)
        robot.update(0.01)
        
        status = robot.get_status()
        assert status['motor_status']['left_pwm'] < 0  # Left wheel reverse
        assert status['motor_status']['right_pwm'] > 0  # Right wheel forward

    def test_lean_turn_reduces_inner_wheel(self, robot):
        """Lean turn should reduce inner wheel speed."""
        robot.accelerate()
        robot.update(0.5)  # Ramp up to L1
        robot.turn_right(intensity=0.6)
        robot.update(0.01)
        
        status = robot.get_status()
        left_pwm = abs(status['motor_status']['left_pwm'])
        right_pwm = abs(status['motor_status']['right_pwm'])
        
        # Right is outer (higher), left is inner (lower)
        assert left_pwm < right_pwm


class TestMotorSignals:
    """Test motor controller receives correct signals."""

    def test_motor_stops_on_stop(self, robot):
        """Motor should receive zero PWM on stop."""
        robot.accelerate()
        robot.update(0.5)
        robot.stop()
        
        status = robot.get_status()
        assert status['motor_status']['left_pwm'] == 0.0
        assert status['motor_status']['right_pwm'] == 0.0

    def test_motor_receives_pwm(self, robot):
        """Motor should receive PWM values from robot."""
        robot.accelerate()
        robot.update(0.5)
        
        status = robot.get_status()
        assert status['motor_status']['left_pwm'] != 0.0
        assert status['motor_status']['right_pwm'] != 0.0


class TestStatusOutput:
    """Test status reporting."""

    def test_get_status_includes_all_fields(self, robot):
        """Status should include all required fields."""
        status = robot.get_status()
        
        required_fields = [
            'state', 'direction', 'speed_level', 'speed_pwm',
            'turn_intensity', 'current_pwm', 'target_pwm', 'motor_status'
        ]
        
        for field in required_fields:
            assert field in status

    def test_status_reflects_current_state(self, robot):
        """Status should reflect actual state."""
        robot.accelerate()
        status = robot.get_status()
        
        assert status['state'] == 'ACCELERATING'
        assert status['direction'] == 'FORWARD'
        assert status['speed_level'] == 'L1'


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
