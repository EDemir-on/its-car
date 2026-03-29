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
    """Thread that reads keyboard input and maintains key state"""
    
    def __init__(self):
        self.command_queue = None
        self.running = False
        self.input_thread = None
        self.fd = None
        self.old_settings = None
        self.lock = threading.Lock()
        # Track which keys are currently pressed
        self.keys_pressed = {'w': False, 's': False, 'a': False, 'd': False}
        self.quit_requested = False
    
    def set_command_queue(self, queue):
        """Set the command queue to send commands to"""
        self.command_queue = queue
    
    def get_key_state(self):
        """Get current state of all keys"""
        with self.lock:
            return dict(self.keys_pressed)
    
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
        """Monitor keyboard input and track key state"""
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
                    
                    with self.lock:
                        if key == 'w':
                            self.keys_pressed['w'] = True
                            if self.command_queue:
                                self.command_queue.put(RobotCommand('key_state_change'))
                        elif key == 's':
                            self.keys_pressed['s'] = True
                            if self.command_queue:
                                self.command_queue.put(RobotCommand('key_state_change'))
                        elif key == 'a':
                            self.keys_pressed['a'] = True
                            if self.command_queue:
                                self.command_queue.put(RobotCommand('key_state_change'))
                        elif key == 'd':
                            self.keys_pressed['d'] = True
                            if self.command_queue:
                                self.command_queue.put(RobotCommand('key_state_change'))
                        elif key == ' ':
                            if self.command_queue:
                                self.command_queue.put(RobotCommand('brake', speed=0.6))
                            # Release all movement keys on brake
                            self.keys_pressed['w'] = False
                            self.keys_pressed['s'] = False
                            self.keys_pressed['a'] = False
                            self.keys_pressed['d'] = False
                        elif key == 'q':
                            log.info("Quit command received")
                            self.quit_requested = True
                            self.running = False
                            break
                        
                        # Handle key release (when user lifts finger, we detect the next key)
                        # This is a limitation of terminal input - we can't detect releases
                        # The control loop will handle timeout-based release
                    
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
