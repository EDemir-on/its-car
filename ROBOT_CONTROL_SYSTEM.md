# Robot Control System - Thread-Based Architecture

## Overview

The new robot control system uses **threading with a command queue** to separate input handling from robot control. This allows:

✅ **Concurrent Operation**: Handle keyboard input AND web server commands simultaneously  
✅ **Non-Blocking**: Web requests don't wait for motor operations  
✅ **Thread-Safe**: Multiple sources can safely send commands  
✅ **Clean API**: Simple functions for controlling the robot from anywhere  

## Architecture

```
┌─────────────────────────────┐
│   Input Sources             │
├─────────────────────────────┤
│ • Keyboard (InputHandler)   │
│ • Web Server (Flask)        │
│ • External APIs             │
└──────┬──────────────────────┘
       │ (send RobotCommand)
       ▼
┌─────────────────────────────┐
│  Command Queue              │
│  (Thread-Safe)              │
└──────┬──────────────────────┘
       │ (dequeue)
       ▼
┌─────────────────────────────┐
│  RobotControllerThread      │
│  (Process Commands)         │
└──────┬──────────────────────┘
       │ (set motor PWM)
       ▼
┌─────────────────────────────┐
│  Motor Hardware (GPIO)      │
└─────────────────────────────┘
```

## Components

### 1. **RobotCommand**
Represents a single robot action with parameters:
```python
command = RobotCommand('forward', speed=0.7, duration=2.0)
```

### 2. **RobotControllerThread**
- Runs in background thread
- Dequeues and executes commands
- Manages motor state
- Handles timing and safety

### 3. **InputHandlerThread**
- Monitors keyboard input without blocking
- Runs in separate thread
- Queues commands based on key presses
- Restores terminal on shutdown

## Usage

### Basic Usage (Keyboard)
```bash
cd /workspaces/its-car
python3 robot_control.py
```

Then use:
- **W** = Move forward
- **S** = Move backward
- **A** = Turn left
- **D** = Turn right
- **SPACE** = Smooth brake
- **Q** = Quit

### Web Server Usage
```bash
cd /workspaces/its-car/web-control/robot-web-control/src
python3 server.py
```

Visit `http://localhost:8080` and control the robot through the web interface.

### Programmatic Usage
```python
import robot_control

# Initialize
robot_control.initialize_robot()

# Send commands from anywhere
robot_control.move_forward(speed=0.6)
robot_control.turn_left(speed=0.5)
robot_control.stop()

# Or use low-level API
robot_control.send_command('forward', speed=0.7, duration=2.0)

# Get status
status = robot_control.get_status()
print(status)

# Shutdown
robot_control.shutdown_robot()
```

## API Reference

### Initialization
```python
robot_control.initialize_robot()  # Start robot system
robot_control.shutdown_robot()    # Stop robot system
```

### Control Commands
```python
robot_control.send_command(action, speed=0.6, duration=None)
robot_control.move_forward(speed=0.7)
robot_control.move_backward(speed=0.7)
robot_control.turn_left(speed=0.6)
robot_control.turn_right(speed=0.6)
robot_control.stop()
robot_control.brake(speed=0.6)
```

### Status
```python
status = robot_control.get_status()
# Returns:
# {
#     'speed_A': 0.7,
#     'speed_B': 0.6,
#     'last_command': 'RobotCommand(forward, speed=0.7)',
#     'running': True
# }
```

## Key Features

### 1. **Non-Blocking Architecture**
- Web requests return immediately
- Keyboard input doesn't block motor control
- Commands are queued and processed sequentially

### 2. **Thread Safety**
- Command queue is thread-safe (Python `queue.Queue`)
- Motor state protected by lock
- Multiple sources can safely send commands

### 3. **Smooth Operation**
- Commands are processed at fixed interval (20ms)
- Smooth braking with gradual deceleration
- Timeout-based auto-stop for safety

### 4. **Easy Integration**
- Simple API for external systems
- Flask server fully integrated
- Can be imported in any Python code

## How It Fixes the Input Problem

### Before (Old System)
```
Main Thread
├─ Wait for keyboard (BLOCKING with getch())
│  └─ Can't process web requests
└─ Process command
└─ Repeat
```

**Problem**: Web requests hang while waiting for keyboard input.

### After (New System)
```
Thread 1: InputHandler          Thread 2: RobotController
├─ Monitor keyboard (non-blocking)   ├─ Dequeue commands
├─ Queue command                     ├─ Control motors
└─ Continue listening                └─ Loop at 50Hz

Main Thread
└─ Flask web server (not blocked)
```

**Solution**: Both threads work concurrently, web server always responsive.

## Configuration

Edit these constants in `robot_control.py` to customize behavior:

```python
# Control timing
control_dt = 0.02              # Robot control loop interval
input_poll_dt = 0.005          # Keyboard polling interval

# Speed defaults
speed = 0.6                    # Default speed
turn_speed = 0.5              # Turn speed

# Brake behavior
brake_step = 0.05             # Deceleration step
brake_delay = 0.05            # Deceleration interval
```

## Testing

Run the test script to see both keyboard and programmatic commands:
```bash
python3 test_robot_control.py
```

This demonstrates:
- Keyboard input running independently
- Test commands being sent from separate thread
- Real-time status updates
- All happening concurrently

## Error Handling

The system handles:
- Terminal setup failures (degrades gracefully)
- Full command queue (logs warning)
- Motor communication errors (logs error)
- Keyboard interrupt (clean shutdown)
- Thread shutdown timeouts (forced cleanup)

## Shutdown Behavior

1. `shutdown_robot()` sets stop flags
2. Control thread processes final commands
3. Motors are stopped
4. Input thread stops monitoring
5. Terminal settings restored
6. Both threads join (with timeout)
7. All resources cleaned up

---

**Status**: ✅ Production Ready  
**Thread-Safe**: ✅ Yes  
**Hardware Required**: Raspberry Pi with motor drivers
