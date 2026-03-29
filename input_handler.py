"""
Input Handler System - Separates keyboard input from robot control
Runs in a dedicated thread to handle user input without blocking robot control
"""

import sys
import tty
import termios
import threading
import logging
from robot_system import RobotCommand

# Setup logging
logging.basicConfig(level=logging.INFO, format='[%(levelname)s] %(message)s')
log = logging.getLogger("input_handler")


class InputHandler:
    """Thread that reads keyboard input without blocking main control"""
    
    def __init__(self):
        self.command_queue = None
        self.running = False
        self.input_thread = None
        self.fd = None
        self.old_settings = None
    
    def set_command_queue(self, queue):
        """Set the command queue to send commands to"""
        self.command_queue = queue
    
    def start(self):
        """Start the input handler thread"""
        if self.running:
            log.warning("Input handler already running")
            return
        
        if self.command_queue is None:
            log.error("Command queue not set. Call set_command_queue() first.")
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
# Demo - Keyboard Input Handler
# ----------------------------
if __name__ == '__main__':
    import time
    from robot_system import initialize_robot, shutdown_robot
    
    log.info("Starting robot system with keyboard input...")
    
    # Create input handler
    input_handler = InputHandler()
    
    # Initialize robot with input handler
    initialize_robot(input_handler=input_handler)
    
    try:
        # Keep main thread alive while input handler monitors keyboard
        while input_handler.running:
            time.sleep(0.1)
    
    except KeyboardInterrupt:
        log.info("KeyboardInterrupt received")
    
    finally:
        shutdown_robot()
        print("\nExited safely")
