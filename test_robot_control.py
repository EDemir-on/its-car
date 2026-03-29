#!/usr/bin/env python3
"""
Test script for the new robot control system
Demonstrates separate input and command handling
"""

import robot_control
import time
import threading
import sys


def print_status():
    """Print robot status periodically"""
    while robot_control._robot_system is not None:
        status = robot_control.get_status()
        print(f"\n📊 Status: {status}", file=sys.stderr)
        time.sleep(1.0)


def send_test_commands():
    """Send some test commands from external source"""
    time.sleep(2)
    
    if robot_control._robot_system is None:
        print("Robot not initialized", file=sys.stderr)
        return
    
    print("\n🤖 Sending test commands (separate from keyboard input)...", file=sys.stderr)
    
    test_commands = [
        ('forward', 0.6, 1.0),
        ('left', 0.5, 1.0),
        ('right', 0.5, 1.0),
        ('backward', 0.6, 1.0),
        ('stop', 0.0, None),
    ]
    
    for action, speed, duration in test_commands:
        if duration:
            print(f"  → {action} at speed {speed} for {duration}s", file=sys.stderr)
        else:
            print(f"  → {action}", file=sys.stderr)
        
        robot_control.send_command(action, speed=speed, duration=duration)
        time.sleep(duration + 0.5) if duration else time.sleep(0.5)
    
    print("\n✅ Test commands complete", file=sys.stderr)


if __name__ == '__main__':
    print("🚀 Robot Control System - Test Mode", file=sys.stderr)
    print("Instructions:", file=sys.stderr)
    print("  • Keyboard input: WASD=move, SPACE=brake, Q=quit", file=sys.stderr)
    print("  • Separate thread: Sending test commands in 2 seconds...", file=sys.stderr)
    
    robot_control.initialize_robot()
    
    # Start status printer
    status_thread = threading.Thread(target=print_status, daemon=True)
    status_thread.start()
    
    # Start test command sender
    test_thread = threading.Thread(target=send_test_commands, daemon=True)
    test_thread.start()
    
    try:
        # Keep main thread alive
        while robot_control._robot_system is not None:
            time.sleep(0.1)
    
    except KeyboardInterrupt:
        print("\nShutdown initiated...", file=sys.stderr)
    
    finally:
        robot_control.shutdown_robot()
        print("\n✨ Robot system stopped safely", file=sys.stderr)
