# GPIO Pin Mapping and Motor Control System

## Complete GPIO Pin Configuration

### Motor A (Left Motor)
- **Control Pins:**
  - `in1` = GPIO 17 (Forward control)
  - `in2` = GPIO 27 (Backward control)
- **PWM Pin:**
  - `pwm` = GPIO 12 (ENA - Motor speed control)

### Motor B (Right Motor) 
- **Control Pins:**
  - `in3` = GPIO 22 (Forward control)
  - `in4` = GPIO 23 (Backward control)
- **PWM Pin:**
  - `pwm` = GPIO 13 (ENB - Motor speed control)

## Motor Driver Type

**Driver Type:** L298N-style H-Bridge (based on GPIO pin configuration)
- Uses 2 digital pins per motor for direction control (forward/backward)
- Uses PWM pins (ENA and ENB) for speed control (0-255 or 0.0-1.0)

## Library & PWM Configuration

**Motor Control Library:** `gpiozero`
```python
from gpiozero import PWMLED, DigitalOutputDevice
```

**PWM Implementation:**
```python
motorA_pwm = PWMLED(12)  # ENA pin
motorB_pwm = PWMLED(13)  # ENB pin
```

**PWM Value Range:** 0.0 to 1.0 (float, representing duty cycle)
- `0.0` = motor off
- `1.0` = motor at full speed
- Intermediate values (e.g., 0.5) = partial speed

**Default PWM Frequency:** gpiozero's PWMLED uses default frequency (typically 100-1000 Hz depending on system)

---

## Motor B Reversed Pinning (CRITICAL)

### **Motor B Control Logic is INVERTED**

Motor B's power pins are physically reversed in the hardware circuit, requiring reversed GPIO control:

```python
invert_B = True  # Flag used throughout implementations
```

### Forward/Backward Direction Comparison

#### Motor A (Standard Control)
```python
# Motor A Forward
motorA_in1.on()   # GPIO 17 = HIGH
motorA_in2.off()  # GPIO 27 = LOW

# Motor A Backward
motorA_in1.off()  # GPIO 17 = LOW
motorA_in2.on()   # GPIO 27 = HIGH
```

#### Motor B (INVERTED Control)
```python
# Motor B Forward (INVERTED pins)
motorB_in3.off()  # GPIO 22 = LOW   (inverted from normal)
motorB_in4.on()   # GPIO 23 = HIGH  (inverted from normal)

# Motor B Backward (INVERTED pins)
motorB_in3.on()   # GPIO 22 = HIGH  (inverted from normal)
motorB_in4.off()  # GPIO 23 = LOW   (inverted from normal)
```

---

## Motor Control Functions

### Function 1: Basic Motor Control (From `robot.py`)

```python
def motor_A(forward=True, speed=0.5):
    """Control left motor (Motor A)"""
    motorA_in1.on() if forward else motorA_in1.off()
    motorA_in2.off() if forward else motorA_in2.on()
    global current_speed_A
    current_speed_A = speed
    motorA_pwm.value = speed  # Write PWM value (0.0-1.0)

def motor_B(forward=True, speed=0.5):
    """Control right motor (Motor B) - WITH INVERSION"""
    motorB_in3.on() if forward else motorB_in3.off()
    motorB_in4.off() if forward else motorB_in4.on()
    global current_speed_B
    current_speed_B = speed
    motorB_pwm.value = speed  # Write PWM value (0.0-1.0)
```

**Note:** The simple `motor_B()` in `robot.py` uses NON-INVERTED control. More recent implementations use INVERTED control.

---

### Function 2: Inverted Motor Control (From `robot_control.py` - Recommended)

```python
def _motor_A(self, forward=True, speed=0.5):
    """Control motor A (left) - Standard"""
    with self.lock:
        motorA_in1.on() if forward else motorA_in1.off()
        motorA_in2.off() if forward else motorA_in2.on()
        self.current_speed_A = speed
        motorA_pwm.value = speed  # Write PWM value

def _motor_B(self, forward=True, speed=0.5):
    """Control motor B (right) - INVERTED due to reversed power pins"""
    with self.lock:
        # Inverted: Motor B has reversed power pins
        motorB_in3.off() if forward else motorB_in3.on()  # INVERTED
        motorB_in4.on() if forward else motorB_in4.off()  # INVERTED
        self.current_speed_B = speed
        motorB_pwm.value = speed  # Write PWM value
```

---

### Function 3: Comprehensive Motor Speed Control with Inversion (From `robot_control.py`)

This function handles both direction and PWM with proper Motor B inversion:

```python
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
        
        # Motor A: Standard H-bridge control
        if speed_A >= 0:
            motorA_in1.on()    # Forward
            motorA_in2.off()
        else:
            motorA_in1.off()   # Backward
            motorA_in2.on()
        motorA_pwm.value = pwm_A  # Write PWM (0.0-1.0)
        
        # Motor B: INVERTED H-bridge control (due to reversed power pins)
        if speed_B >= 0:
            motorB_in3.off()   # Forward (INVERTED)
            motorB_in4.on()
        else:
            motorB_in3.on()    # Backward (INVERTED)
            motorB_in4.off()
        motorB_pwm.value = pwm_B  # Write PWM (0.0-1.0)
```

---

### Function 4: Alternative Inverted Control Based on Direction (From `robot3.py`)

```python
def set_motor_direction(left_forward, right_forward):
    """Set motor direction with Motor B inversion"""
    motorA_in1.on() if left_forward else motorA_in1.off()
    motorA_in2.off() if left_forward else motorA_in2.on()
    
    # Motor B inversion: effective_right differs from requested right_forward
    effective_right = (right_forward != invert_B)  # invert_B = True
    motorB_in3.on() if effective_right else motorB_in3.off()
    motorB_in4.off() if effective_right else motorB_in4.on()

def command_motor_signed(left_cmd, right_cmd):
    """Control motors with signed values (-1.0 to 1.0)"""
    left_cmd = clamp(left_cmd, -1.0, 1.0)
    right_cmd = clamp(right_cmd, -1.0, 1.0)
    
    if abs(left_cmd) < deadband and abs(right_cmd) < deadband:
        stop_motors()
        return
    
    # Set direction based on sign
    set_motor_direction(left_cmd >= 0.0, right_cmd >= 0.0)
    
    # Write PWM values (absolute values, direction already set)
    motorA_pwm.value = abs(left_cmd)  # Write PWM
    motorB_pwm.value = abs(right_cmd)  # Write PWM
```

---

## PWM Write Methods Summary

All implementations use one of these PWM write methods:

### Method 1: Direct PWM Assignment (gpiozero)
```python
# Motor A
motorA_pwm.value = 0.5  # Set to 50% speed

# Motor B
motorB_pwm.value = 0.5  # Set to 50% speed
```

### Method 2: Within Lock for Thread Safety
```python
with self.lock:
    motorA_pwm.value = speed  # Thread-safe PWM write
    motorB_pwm.value = speed  # Thread-safe PWM write
```

**Value Range:** 0.0 (stop) to 1.0 (full speed)
**Type:** Float (decimal)

---

## Stop Motor Function (Emergency Stop)

```python
def _stop_motors(self):
    """Immediately stop both motors"""
    with self.lock:
        motorA_pwm.value = 0    # Stop Motor A
        motorB_pwm.value = 0    # Stop Motor B
        motorA_in1.off()        # Deactivate all control pins
        motorA_in2.off()
        motorB_in3.off()
        motorB_in4.off()
        self.current_speed_A = 0
        self.current_speed_B = 0
```

---

## Example Usage: Forward Motion

```python
# Using _motor_A and _motor_B methods
speed = 0.7  # 70% speed

# Move both motors forward
_motor_A(forward=True, speed=speed)   # Left: GPIO 17 = HIGH, GPIO 27 = LOW, ENA = 0.7
_motor_B(forward=True, speed=speed)   # Right: GPIO 22 = LOW (inverted), GPIO 23 = HIGH (inverted), ENB = 0.7

# Result: Both motors spin forward at 70% speed
```

---

## Example Usage: Turn Right

```python
# Left motor at full speed, right motor at reduced speed
speed = 0.7
turn_reduction = 0.5

_motor_A(forward=True, speed=speed)              # Left at 70%
_motor_B(forward=True, speed=speed * turn_reduction)  # Right at 35%

# Result: Robot curves right
```

---

## Hardware Circuit Summary

| Component | GPIO | Function |
|-----------|------|----------|
| Motor A (Left) in1 | 17 | Forward control |
| Motor A (Left) in2 | 27 | Backward control |
| Motor A PWM (ENA) | 12 | Speed control |
| Motor B (Right) in3 | 22 | Control (INVERTED) |
| Motor B (Right) in4 | 23 | Control (INVERTED) |
| Motor B PWM (ENB) | 13 | Speed control |

---

## Key Implementation Notes

1. **Motor B Inversion is Critical:** All direction pins for Motor B are inverted in code compared to Motor A
2. **PWM Thread Safety:** When multiple threads access motors, use locks
3. **Minimum PWM Threshold:** Some implementations enforce min_pwm (e.g., 0.3) to avoid motor stall
4. **Smooth Braking:** Many implementations support gradual speed reduction
5. **State Machine Approach:** More advanced implementations use state machines for acceleration/deceleration control
