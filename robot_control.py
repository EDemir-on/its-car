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
    """Thread-safe robot controller that processes commands from a queue"""
    
    def __init__(self, command_queue, poll_interval=0.02):
        self.command_queue = command_queue
        self.poll_interval = poll_interval
        self.running = False
        self.lock = threading.Lock()
        
        # Motor state
        self.current_speed_A = 0
        self.current_speed_B = 0
        self.command_timeout = None
        self.last_command = None
        
    def start(self):
        """Start the robot control thread"""
        if self.running:
            log.warning("Robot controller already running")
            return
        
        self.running = True
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
        """Main robot control loop - processes commands from queue"""
        try:
            while self.running:
                try:
                    # Try to get a command with timeout to allow checking running flag
                    command = self.command_queue.get(timeout=self.poll_interval)
                    self.last_command = command
                    self._execute_command(command)
                    
                except queue.Empty:
                    # No command, check for timeout and stop if needed
                    if self.command_timeout and time.time() > self.command_timeout:
                        self._stop_motors()
                        self.command_timeout = None
                    time.sleep(self.poll_interval)
                    
        except Exception as e:
            log.error(f"Control loop error: {e}", exc_info=True)
        finally:
            self._stop_motors()
    
    def _execute_command(self, cmd):
        """Execute a single command"""
        try:
            log.debug(f"Executing: {cmd}")
            
            if cmd.action == 'forward':
                self._motor_A(True, cmd.speed)
                self._motor_B(True, cmd.speed)
                
            elif cmd.action == 'backward':
                self._motor_A(False, cmd.speed)
                self._motor_B(False, cmd.speed)
                
            elif cmd.action == 'left':
                # Turn left: right wheel faster
                self._motor_A(False, cmd.speed * 0.7)
                self._motor_B(True, cmd.speed)
                
            elif cmd.action == 'right':
                # Turn right: left wheel faster
                self._motor_A(True, cmd.speed)
                self._motor_B(False, cmd.speed * 0.7)
                
            elif cmd.action == 'stop':
                self._stop_motors()
                
            elif cmd.action == 'brake':
                self._smooth_brake(cmd.speed)
            
            # Set timeout for auto-stop if needed
            if cmd.duration:
                self.command_timeout = time.time() + cmd.duration
            else:
                self.command_timeout = None
                
        except Exception as e:
            log.error(f"Command execution error: {e}", exc_info=True)
            self._stop_motors()
    
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
            motorB_in3.on() if forward else motorB_in3.off()
            motorB_in4.off() if forward else motorB_in4.on()
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
    """Thread that reads keyboard input without blocking main control"""
    
    def __init__(self, command_queue):
        self.command_queue = command_queue
        self.running = False
        self.input_thread = None
        self.fd = None
        self.old_settings = None
    
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
        """Monitor keyboard input and queue commands"""
        try:
            log.info("Input handler ready: WASD=move, SPACE=brake, Q=quit")
            
            while self.running:
                try:
                    # Non-blocking read with small timeout
                    import select
                    dr, _, _ = select.select([sys.stdin], [], [], 0.05)
                    
                    if not dr:
                        continue
                    
                    key = sys.stdin.read(1).lower()
                    
                    if key == 'w':
                        self.command_queue.put(RobotCommand('forward', speed=0.7))
                    elif key == 's':
                        self.command_queue.put(RobotCommand('backward', speed=0.7))
                    elif key == 'a':
                        self.command_queue.put(RobotCommand('left', speed=0.6))
                    elif key == 'd':
                        self.command_queue.put(RobotCommand('right', speed=0.6))
                    elif key == ' ':
                        self.command_queue.put(RobotCommand('brake', speed=0.6))
                    elif key == 'q':
                        log.info("Quit command received")
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
        
        # Create control thread
        controller = RobotControllerThread(cmd_queue)
        controller.start()
        
        # Create input handler thread
        input_handler = InputHandlerThread(cmd_queue)
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
