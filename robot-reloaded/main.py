"""
Demo script for robot movement system.
Single cycle with timed state transitions and graceful shutdown.
"""

import logging
import signal
import sys
import time
from pathlib import Path

from state_definitions import MovementState, Direction
from motor_controller import MotorController
from robot_movement import RobotMovement


# ========== Logging Setup ==========

def setup_logging():
    """Configure logging to file and console."""
    log_dir = Path(__file__).parent / "logs"
    log_dir.mkdir(parents=True, exist_ok=True)
    log_file = log_dir / "robot_movement.log"
    
    # Create logger
    logger = logging.getLogger()
    logger.setLevel(logging.DEBUG)
    
    # File handler
    file_handler = logging.FileHandler(log_file)
    file_handler.setLevel(logging.DEBUG)
    file_formatter = logging.Formatter(
        "%(asctime)s | %(name)s | %(levelname)-8s | %(message)s",
        datefmt="%Y-%m-%d %H:%M:%S"
    )
    file_handler.setFormatter(file_formatter)
    
    # Console handler
    console_handler = logging.StreamHandler()
    console_handler.setLevel(logging.INFO)
    console_formatter = logging.Formatter(
        "%(levelname)-8s | %(message)s"
    )
    console_handler.setFormatter(console_formatter)
    
    logger.addHandler(file_handler)
    logger.addHandler(console_handler)
    
    return logger


# ========== Global Variables ==========

logger = None
robot = None


# ========== Signal Handler ==========

def signal_handler(signum, frame):
    """Handle Ctrl+C gracefully."""
    logger.warning("=" * 80)
    logger.warning("INTERRUPT RECEIVED (Ctrl+C) - SHUTTING DOWN SAFELY")
    logger.warning("=" * 80)
    
    if robot:
        logger.info("Stopping robot movement...")
        robot.stop()  # Ensure all motors stop
    
    logger.info("Shutdown complete")
    sys.exit(0)


# ========== Demo Sequence ==========

def run_demo_sequence():
    """Run a single complete demo cycle with state transitions."""
    
    logger.info("=" * 80)
    logger.info("ROBOT MOVEMENT DEMO - SINGLE CYCLE")
    logger.info("=" * 80)
    logger.info("Sequence: STATIONARY → ACCELERATING → COASTING → STATIONARY_TURN → MOVING_TURN → BRAKING → STATIONARY")
    logger.info("Press Ctrl+C to stop at any time")
    logger.info("=" * 80)
    
    # Demo sequence: list of (action, duration_seconds)
    sequence = [
        ("STATIONARY_WAIT", 2.0),
        ("ACCELERATE", 3.0),
        ("STATIONARY_WAIT", 2.0),
        ("COAST", 3.0),
        ("STATIONARY_WAIT", 2.0),
        ("TURN_LEFT", 3.0),
        ("STATIONARY_WAIT", 2.0),
        ("TURN_RIGHT", 3.0),
        ("STATIONARY_WAIT", 2.0),
        ("BRAKE", 2.0),
        ("STATIONARY_WAIT", 2.0),
    ]
    
    # Run each phase
    for phase_num, (action, duration) in enumerate(sequence, 1):
        logger.info(f"\n--- Phase {phase_num}/{len(sequence)}: {action} ({duration:.1f}s) ---")
        
        # Execute action
        if action == "STATIONARY_WAIT":
            # Already in stationary, just wait
            pass
        elif action == "ACCELERATE":
            robot.accelerate()
        elif action == "COAST":
            robot.coast()
        elif action == "BRAKE":
            robot.brake()
        elif action == "TURN_LEFT":
            # Turn left while moving (robot should be in COASTING state)
            robot.turn_left(intensity=-0.6)
        elif action == "TURN_RIGHT":
            # First stop, then go stationary
            robot.stop()
            time.sleep(0.1)
            # Now turn right (will be stationary pivot)
            robot.turn_right(intensity=0.6)
        
        # Run update loop for the duration
        phase_start = time.time()
        update_interval = 0.01  # 10ms = 100 Hz
        
        while time.time() - phase_start < duration:
            delta = update_interval
            robot.update(delta)
            
            # Print status every 1 second
            if int((time.time() - phase_start) * 1000) % 1000 < update_interval * 1000:
                status = robot.get_status()
                logger.info(
                    f"  State: {status['state']:15} | Direction: {status['direction']:7} | "
                    f"Speed: {status['speed_level']:2} | Turn: {status['turn_intensity']:+.2f} | "
                    f"PWM: {status['current_pwm']:.3f}→{status['target_pwm']:.3f}"
                )
            
            time.sleep(update_interval)
    
    # Final stop
    logger.info("\n--- Final: STOP ---")
    robot.stop()
    time.sleep(0.1)
    
    logger.info("=" * 80)
    logger.info("DEMO COMPLETE - Robot returned to STATIONARY")
    logger.info("=" * 80)


def main():
    """Main entry point."""
    global logger, robot
    
    # Setup logging
    logger = setup_logging()
    logger.info("Robot Movement Demo Starting...")
    
    try:
        # Register signal handler for Ctrl+C
        signal.signal(signal.SIGINT, signal_handler)
        
        # Initialize motor controller (mock mode)
        logger.info("Initializing motor controller (mock mode)...")
        motor_controller = MotorController(use_mock=True)
        
        # Initialize robot movement system
        logger.info("Initializing robot movement system...")
        robot = RobotMovement(motor_controller=motor_controller)
        
        # Run demo
        run_demo_sequence()
        
    except Exception as e:
        logger.exception(f"Error during demo: {e}")
        sys.exit(1)
    finally:
        # Cleanup
        if robot:
            logger.info("Performing final cleanup...")
            robot.stop()
        if motor_controller:
            motor_controller.close()
        logger.info("Demo ended")


if __name__ == "__main__":
    main()
