"""
State definitions and enums for robot movement system.
"""

from enum import Enum, auto


class MovementState(Enum):
    """Movement states of the robot."""
    STATIONARY = auto()
    ACCELERATING = auto()
    COASTING = auto()
    BRAKING = auto()
    MOVING_TURN = auto()
    STATIONARY_TURN = auto()


class Direction(Enum):
    """Direction of movement."""
    FORWARD = auto()
    REVERSE = auto()
    NEUTRAL = auto()


class SpeedLevel(Enum):
    """Discrete speed levels (PWM percentage)."""
    L0 = 0.0      # 0% - Stationary
    L1 = 0.4      # 40%
    L2 = 0.7      # 70%
    L3 = 1.0      # 100%


class TurnIntensity(Enum):
    """Turn intensity (angle representation)."""
    STRAIGHT = 0.0           # 0° - No turn
    GENTLE_LEFT = 0.3        # ~30° left
    GENTLE_RIGHT = -0.3      # ~30° right
    MEDIUM_LEFT = 0.6        # ~60° left
    MEDIUM_RIGHT = -0.6      # ~60° right
    SHARP_LEFT = 1.0         # ~90° left (pivot)
    SHARP_RIGHT = -1.0       # ~90° right (pivot)


# PWM Ramping Configuration
RAMP_TIME_ACCELERATE = 0.5  # seconds to ramp to next speed level
RAMP_TIME_BRAKE = 0.3       # seconds to decelerate to stop
MIN_PWM = 0.0               # Minimum PWM to avoid motor stall
MIN_SPEED_FOR_MOVEMENT = 0.1  # Minimum speed level to be considered "moving"

# Turning Configuration
TURN_RATIO_MOVING = 0.5       # Inner wheel speed when turning while moving (50% of outer)
TURN_RATIO_GENTLE = 0.7       # Higher ratio for gentler turns (70% of outer)

# Motor Safety
DIRECTION_CHANGE_NEUTRAL_TIME = 0.2  # seconds to wait before reversing
