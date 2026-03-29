"""
Robot Control System with Thread-Safe Command Queue
Separates input handling from robot control using threading.
"""

import threading
import queue
import time
import sys
import tty
import termios
import logging
from gpiozero import PWMLED, DigitalOutputDevice

# Setup logging
logging.basicConfig(level=logging.INFO, format='[%(levelname)s] %(message)s')
log = logging.getLogger("robot_control")

# ----------------------------
# Motor Hardware Setup
# ----------------------------
motorA_in1 = DigitalOutputDevice(17)
motorA_in2 = DigitalOutputDevice(27)
motorB_in3 = DigitalOutputDevice(22)
motorB_in4 = DigitalOutputDevice(23)
motorA_pwm = PWMLED(12)  # ENA
motorB_pwm = PWMLED(13)  # ENB

# ----------------------------
# Command Queue System
# ----------------------------
class RobotCommand:
    """Represents a single robot command"""
    def __init__(self, action, **kwargs):
        self.action = action  # 'forward', 'backward', 'left', 'right', 'stop', 'brake'
        self.speed = kwargs.get('speed', 0.6)
        self.duration = kwargs.get('duration', None)
        self.timestamp = time.time()
    
    def __repr__(self):
        return f"RobotCommand({self.action}, speed={self.speed}, duration={self.duration})"


class RobotControllerThread:
    """Thread-safe robot controller with explicit state machine"""
    
    # State machine states
    STATE_IDLE = 0
    STATE_ACCELERATING = 1
    STATE_COASTING = 2
    STATE_MOVING_TURN = 3
    STATE_STATIONARY_TURN = 4
    
    def __init__(self, command_queue, input_handler=None, poll_interval=0.02):
        self.command_queue = command_queue
        self.input_handler = input_handler  # Reference to input handler for key state
        self.poll_interval = poll_interval
        self.running = False
        self.lock = threading.Lock()
        
        # Motor state
        self.current_speed_A = 0.0
        self.current_speed_B = 0.0
        self.current_state = self.STATE_IDLE
        self.command_timeout = None
        self.last_command = None
        
        # Control parameters
        self.max_speed_forward = 0.7
        self.max_speed_backward = 0.5
        self.min_pwm = 0.3
        self.acceleration = 0.4  # Faster acceleration
        self.deceleration = 0.15  # Slower coasting deceleration
        self.stationary_turn_speed = 0.5  # Speed for in-place turning
        self.turn_reduction = 0.5  # Reduce inner wheel to 50% during moving turns
        
        self.last_update_time = time.time()
        
    def start(self):
        """Start the robot control thread"""
        if self.running:
            log.warning("Robot controller already running")
            return
        
        self.running = True
        self.last_update_time = time.time()
        self.control_thread = threading.Thread(target=self._control_loop, daemon=False)
        self.control_thread.start()
        log.info("Robot control thread started")
    
    def stop(self):
        """Stop the robot control thread"""
        self.running = False
        if hasattr(self, 'control_thread'):
            self.control_thread.join(timeout=2.0)
        log.info("Robot control thread stopped")
    
    def _control_loop(self):
        """Main robot control loop - state machine based"""
        try:
            while self.running:
                now = time.time()
                dt = now - self.last_update_time
                self.last_update_time = now
                
                if self.input_handler:
                    self._update_state_machine(dt)
                else:
                    self._update_motors_from_queue(dt)
                
                time.sleep(self.poll_interval)
                    
        except Exception as e:
            log.error(f"Control loop error: {e}", exc_info=True)
        finally:
            self._stop_motors()
    
    def _update_state_machine(self, dt):
        """Update motors using explicit state machine"""
        try:
            # Check for emergency brake
            if self.input_handler.is_brake_pressed():
                self._stop_motors()
                self.current_state = self.STATE_IDLE
                return
            
            key_state = self.input_handler.get_key_state()
            w_pressed = key_state.get('w', False)
            s_pressed = key_state.get('s', False)
            a_pressed = key_state.get('a', False)
            d_pressed = key_state.get('d', False)
            
            # Resolve conflicting inputs
            forward_intent = w_pressed and not s_pressed
            backward_intent = s_pressed and not w_pressed
            left_intent = a_pressed and not d_pressed
            right_intent = d_pressed and not a_pressed
            
            # Get current velocity magnitude
            velocity = max(abs(self.current_speed_A), abs(self.current_speed_B))
            
            # STATE MACHINE
            if self.current_state == self.STATE_IDLE:
                if forward_intent:
                    self.current_state = self.STATE_ACCELERATING
                    self._accelerate(self.max_speed_forward, dt)
                elif backward_intent:
                    self.current_state = self.STATE_ACCELERATING
                    self._accelerate(-self.max_speed_backward, dt)
                elif left_intent or right_intent:
                    self.current_state = self.STATE_STATIONARY_TURN
                    self._stationary_turn(left_intent, dt)
                else:
                    self._apply_motor_speeds(0, 0)
            
            elif self.current_state == self.STATE_ACCELERATING:
                if not forward_intent and not backward_intent:
                    # Released movement key - transition to Coasting
                    self.current_state = self.STATE_COASTING
                elif (left_intent or right_intent):
                    # Add turning while accelerating - transition to Moving Turn
                    self.current_state = self.STATE_MOVING_TURN
                
                # Continue accelerating
                if forward_intent:
                    self._accelerate(self.max_speed_forward, dt)
                elif backward_intent:
                    self._accelerate(-self.max_speed_backward, dt)
                else:
                    # This shouldn't happen, but handle it
                    self.current_state = self.STATE_COASTING
            
            elif self.current_state == self.STATE_MOVING_TURN:
                if not forward_intent and not backward_intent:
                    # Movement key released
                    self.current_state = self.STATE_COASTING
                elif not (left_intent or right_intent):
                    # Turn key released but still accelerating
                    self.current_state = self.STATE_ACCELERATING
                
                # Update position
                if forward_intent:
                    self._accelerate(self.max_speed_forward, dt)
                    if left_intent:
                        self._apply_turn_reduction('left')
                    elif right_intent:
                        self._apply_turn_reduction('right')
                elif backward_intent:
                    self._accelerate(-self.max_speed_backward, dt)
                    if left_intent:
                        self._apply_turn_reduction('left')
                    elif right_intent:
                        self._apply_turn_reduction('right')
            
            elif self.current_state == self.STATE_COASTING:
                if velocity < 0.05:
                    # Stopped - transition to Idle
                    self.current_state = self.STATE_IDLE
                    self._apply_motor_speeds(0, 0)
                elif forward_intent or backward_intent:
                    # Accelerate again
                    self.current_state = self.STATE_ACCELERATING
                elif left_intent or right_intent:
                    # Start turning while coasting
                    self.current_state = self.STATE_STATIONARY_TURN
                else:
                    # Continue coasting - decelerate
                    self._decelerate(dt)
            
            elif self.current_state == self.STATE_STATIONARY_TURN:
                if not (left_intent or right_intent):
                    # Released turn key
                    if velocity < 0.05:
                        self.current_state = self.STATE_IDLE
                        self._apply_motor_speeds(0, 0)
                    else:
                        self.current_state = self.STATE_COASTING
                elif forward_intent or backward_intent:
                    # Started moving while turning
                    self.current_state = self.STATE_MOVING_TURN
                else:
                    # Continue stationary turn
                    self._stationary_turn(left_intent, dt)
        
        except Exception as e:
            log.error(f"State machine error: {e}", exc_info=True)
    
    def _accelerate(self, target_speed, dt):
        """Accelerate toward target speed"""
        speed_diff = target_speed - self.current_speed_A
        if abs(speed_diff) < 0.01:
            desired_speed = target_speed
        elif speed_diff > 0:
            accel = min(speed_diff, self.acceleration * dt)
            desired_speed = self.current_speed_A + accel
        else:
            # Decelerate (or switch direction)
            decel = min(-speed_diff, self.acceleration * dt)
            desired_speed = self.current_speed_A - decel
        
        self._apply_motor_speeds(desired_speed, desired_speed)
    
    def _decelerate(self, dt):
        """Decelerate due to coasting"""
        decel_amount = self.deceleration * dt
        
        new_speed_A = self.current_speed_A
        new_speed_B = self.current_speed_B
        
        if abs(new_speed_A) > decel_amount:
            new_speed_A -= decel_amount if new_speed_A > 0 else -decel_amount
        else:
            new_speed_A = 0
        
        if abs(new_speed_B) > decel_amount:
            new_speed_B -= decel_amount if new_speed_B > 0 else -decel_amount
        else:
            new_speed_B = 0
        
        self._apply_motor_speeds(new_speed_A, new_speed_B)
    
    def _apply_turn_reduction(self, direction):
        """Apply turn reduction to current movement"""
        if direction == 'left':
            # Reduce left wheel speed (Motor A)
            self.current_speed_A *= self.turn_reduction
        else:
            # Reduce right wheel speed (Motor B)
            self.current_speed_B *= self.turn_reduction
        
        self._apply_motor_speeds(self.current_speed_A, self.current_speed_B)
    
    def _stationary_turn(self, left_intent, dt):
        """Rotate in place"""
        if left_intent:
            target_speed_A = -self.stationary_turn_speed
            target_speed_B = self.stationary_turn_speed
        else:  # right_intent
            target_speed_A = self.stationary_turn_speed
            target_speed_B = -self.stationary_turn_speed
        
        self._apply_motor_speeds(target_speed_A, target_speed_B)
    
    def _update_motors_from_queue(self, dt):
        """Update motors from command queue (backward compatibility)"""
        try:
            # Try to get a command with timeout
            command = self.command_queue.get(timeout=self.poll_interval)
            self.last_command = command
            
            # Handle brake command
            if command.action == 'brake':
                self._decelerate_smoothly(dt)
            # For other commands, just decelerate
            else:
                self._decelerate_smoothly(dt)
            
        except queue.Empty:
            # No command, decelerate gradually
            self._decelerate_smoothly(dt)
    
    def _apply_motor_speeds(self, speed_A, speed_B):
        """Apply motor speeds, handling forward/backward and motor B inversion"""
        with self.lock:
            self.current_speed_A = speed_A
            self.current_speed_B = speed_B
            
            # Apply minimum PWM threshold - if moving, ensure minimum speed
            pwm_A = abs(speed_A)
            pwm_B = abs(speed_B)
            
            if 0 < pwm_A < self.min_pwm:
                pwm_A = self.min_pwm
            if 0 < pwm_B < self.min_pwm:
                pwm_B = self.min_pwm
            
            # Motor A
            if speed_A >= 0:
                motorA_in1.on()
                motorA_in2.off()
            else:
                motorA_in1.off()
                motorA_in2.on()
            motorA_pwm.value = pwm_A
            
            # Motor B (with inverted direction control due to reversed power pins)
            if speed_B >= 0:
                motorB_in3.off()  # Inverted
                motorB_in4.on()   # Inverted
            else:
                motorB_in3.on()   # Inverted
                motorB_in4.off()  # Inverted
            motorB_pwm.value = pwm_B
    
    def _decelerate_smoothly(self, dt):
        """Gradually decelerate to a stop"""
        decel_amount = self.deceleration * dt
        
        with self.lock:
            if abs(self.current_speed_A) > decel_amount:
                if self.current_speed_A > 0:
                    self.current_speed_A -= decel_amount
                else:
                    self.current_speed_A += decel_amount
            else:
                self.current_speed_A = 0
            
            if abs(self.current_speed_B) > decel_amount:
                if self.current_speed_B > 0:
                    self.current_speed_B -= decel_amount
                else:
                    self.current_speed_B += decel_amount
            else:
                self.current_speed_B = 0
            
            # Apply minimum PWM threshold
            pwm_A = abs(self.current_speed_A)
            pwm_B = abs(self.current_speed_B)
            
            if 0 < pwm_A < self.min_pwm:
                pwm_A = self.min_pwm
            if 0 < pwm_B < self.min_pwm:
                pwm_B = self.min_pwm
            
            # Update motor pins
            if self.current_speed_A >= 0:
                motorA_in1.on()
                motorA_in2.off()
            else:
                motorA_in1.off()
                motorA_in2.on()
            motorA_pwm.value = pwm_A
            
            if self.current_speed_B >= 0:
                motorB_in3.off()  # Inverted
                motorB_in4.on()   # Inverted
            else:
                motorB_in3.on()   # Inverted
                motorB_in4.off()  # Inverted
            motorB_pwm.value = pwm_B
    
    def _motor_A(self, forward=True, speed=0.5):
        """Control motor A (left)"""
        with self.lock:
            motorA_in1.on() if forward else motorA_in1.off()
            motorA_in2.off() if forward else motorA_in2.on()
            self.current_speed_A = speed
            motorA_pwm.value = speed
    
    def _motor_B(self, forward=True, speed=0.5):
        """Control motor B (right)"""
        with self.lock:
            # Inverted: Motor B has reversed power pins
            motorB_in3.off() if forward else motorB_in3.on()
            motorB_in4.on() if forward else motorB_in4.off()
            self.current_speed_B = speed
            motorB_pwm.value = speed
    
    def _stop_motors(self):
        """Immediately stop both motors"""
        with self.lock:
            motorA_pwm.value = 0
            motorB_pwm.value = 0
            motorA_in1.off()
            motorA_in2.off()
            motorB_in3.off()
            motorB_in4.off()
            self.current_speed_A = 0
            self.current_speed_B = 0
    
    def _smooth_brake(self, speed=0.6, step=0.05, delay=0.05):
        """Gradually decelerate to stop"""
        with self.lock:
            current_A = self.current_speed_A
            current_B = self.current_speed_B
        
        while current_A > 0 or current_B > 0:
            if current_A > 0:
                current_A = max(0, current_A - step)
                motorA_pwm.value = current_A
            if current_B > 0:
                current_B = max(0, current_B - step)
                motorB_pwm.value = current_B
            time.sleep(delay)
        
        self._stop_motors()
    
    def get_status(self):
        """Get current robot status"""
        with self.lock:
            return {
                'speed_A': self.current_speed_A,
                'speed_B': self.current_speed_B,
                'last_command': str(self.last_command),
                'running': self.running
            }


class InputHandlerThread:
    """Thread that reads keyboard input and maintains key state for state-based control"""
    
    def __init__(self, command_queue):
        self.command_queue = command_queue
        self.running = False
        self.input_thread = None
        self.fd = None
        self.old_settings = None
        self.lock = threading.Lock()
        # Track which keys are currently pressed
        self.keys_pressed = {'w': False, 's': False, 'a': False, 'd': False}
        self.quit_requested = False
        self.brake_pressed = False  # SPACE bar for emergency stop
    
    def get_key_state(self):
        """Get current state of all keys"""
        with self.lock:
            return dict(self.keys_pressed)
    
    def is_brake_pressed(self):
        """Check if brake (space) was pressed"""
        with self.lock:
            if self.brake_pressed:
                self.brake_pressed = False  # Reset flag after reading
                return True
            return False
    
    def start(self):
        """Start the input handler thread"""
        if self.running:
            log.warning("Input handler already running")
            return
        
        # Setup terminal for non-blocking input
        try:
            self.fd = sys.stdin.fileno()
            self.old_settings = termios.tcgetattr(self.fd)
            tty.setcbreak(self.fd)
        except Exception as e:
            log.warning(f"Terminal setup failed: {e}")
        
        self.running = True
        self.input_thread = threading.Thread(target=self._input_loop, daemon=False)
        self.input_thread.start()
        log.info("Input handler thread started")
    
    def stop(self):
        """Stop the input handler thread"""
        self.running = False
        if self.input_thread:
            self.input_thread.join(timeout=1.0)
        
        # Restore terminal settings
        if self.fd is not None and self.old_settings is not None:
            try:
                termios.tcsetattr(self.fd, termios.TCSADRAIN, self.old_settings)
            except Exception:
                pass
        
        log.info("Input handler thread stopped")
    
    def _input_loop(self):
        """Monitor keyboard input and track key state"""
        try:
            log.info("Input handler ready: WASD=move, SPACE=stop, Q=quit")
            
            while self.running:
                try:
                    # Non-blocking read with small timeout
                    import select
                    dr, _, _ = select.select([sys.stdin], [], [], 0.05)
                    
                    if not dr:
                        continue
                    
                    key = sys.stdin.read(1).lower()
                    
                    with self.lock:
                        if key == 'w':
                            self.keys_pressed['w'] = True
                        elif key == 's':
                            self.keys_pressed['s'] = True
                        elif key == 'a':
                            self.keys_pressed['a'] = True
                        elif key == 'd':
                            self.keys_pressed['d'] = True
                        elif key == ' ':
                            # SPACE bar: stop everything
                            self.brake_pressed = True
                            self.keys_pressed['w'] = False
                            self.keys_pressed['s'] = False
                            self.keys_pressed['a'] = False
                            self.keys_pressed['d'] = False
                        elif key == 'q':
                            log.info("Quit command received")
                            self.quit_requested = True
                            self.running = False
                            break
                    
                except Exception as e:
                    log.error(f"Input read error: {e}")
                    continue
        
        finally:
            log.info("Input loop ended")


# ----------------------------
# Global Robot System
# ----------------------------
_robot_system = None
_system_lock = threading.Lock()


def initialize_robot():
    """Initialize and start the robot control system"""
    global _robot_system
    
    with _system_lock:
        if _robot_system is not None:
            log.warning("Robot already initialized")
            return _robot_system
        
        # Create command queue
        cmd_queue = queue.Queue(maxsize=100)
        
        # Create input handler thread first
        input_handler = InputHandlerThread(cmd_queue)
        input_handler.start()
        
        # Create control thread with reference to input handler for state-based control
        controller = RobotControllerThread(cmd_queue, input_handler=input_handler)
        controller.start()
        input_handler.start()
        
        _robot_system = {
            'queue': cmd_queue,
            'controller': controller,
            'input_handler': input_handler
        }
        
        log.info("Robot system initialized")
        return _robot_system


def shutdown_robot():
    """Shutdown the robot control system"""
    global _robot_system
    
    with _system_lock:
        if _robot_system is None:
            return
        
        _robot_system['controller'].stop()
        _robot_system['input_handler'].stop()
        _robot_system = None
        log.info("Robot system shutdown")


def send_command(action, speed=0.6, duration=None):
    """
    Send a command to the robot from external sources (web, etc.)
    
    Args:
        action: 'forward', 'backward', 'left', 'right', 'stop', 'brake'
        speed: 0.0-1.0
        duration: Optional time in seconds to hold command
    
    Returns:
        bool: True if command queued successfully
    """
    if _robot_system is None:
        log.warning("Robot not initialized")
        return False
    
    try:
        cmd = RobotCommand(action, speed=speed, duration=duration)
        _robot_system['queue'].put_nowait(cmd)
        log.info(f"Command queued: {cmd}")
        return True
    except queue.Full:
        log.error("Command queue full")
        return False
    except Exception as e:
        log.error(f"Failed to queue command: {e}")
        return False


def move_forward(speed=0.7):
    """Move forward"""
    return send_command('forward', speed=speed)


def move_backward(speed=0.7):
    """Move backward"""
    return send_command('backward', speed=speed)


def turn_left(speed=0.6):
    """Turn left"""
    return send_command('left', speed=speed)


def turn_right(speed=0.6):
    """Turn right"""
    return send_command('right', speed=speed)


def stop():
    """Stop motors immediately"""
    return send_command('stop')


def brake(speed=0.6):
    """Smooth brake"""
    return send_command('brake', speed=speed)


def get_status():
    """Get robot status"""
    if _robot_system is None:
        return {'status': 'not_initialized'}
    return _robot_system['controller'].get_status()


# ----------------------------
# Main Entry Point
# ----------------------------
if __name__ == '__main__':
    log.info("Starting robot control system...")
    
    initialize_robot()
    
    try:
        # Keep main thread alive while input handler monitors keyboard
        while _robot_system is not None and _robot_system['input_handler'].running:
            time.sleep(0.1)
    
    except KeyboardInterrupt:
        log.info("KeyboardInterrupt received")
    
    finally:
        shutdown_robot()
        print("\nExited safely")
