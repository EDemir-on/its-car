from flask import Flask, render_template, request
import logging
import sys
import os

# Add parent directory to path to import robot modules
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '../..'))

import robot_system

app = Flask(__name__)
app.logger.setLevel(logging.INFO)

# Initialize the robot control system (without input handler for web control)
try:
    robot_system.initialize_robot(input_handler=None)
    app.logger.info("Robot control system initialized")
except Exception as e:
    app.logger.error(f"Failed to initialize robot control: {e}")

def _call_robot_move(direction):
    """Send command to robot control system"""
    d = (direction or "").strip().lower()
    
    action_map = {
        'w': ('forward', 0.7),
        'forward': ('forward', 0.7),
        's': ('backward', 0.7),
        'back': ('backward', 0.7),
        'reverse': ('backward', 0.7),
        'a': ('left', 0.6),
        'left': ('left', 0.6),
        'd': ('right', 0.6),
        'right': ('right', 0.6),
        'stop': ('stop', 0),
        'space': ('brake', 0.6),
        'brake': ('brake', 0.6),
    }
    
    if d not in action_map:
        raise AttributeError(f"Unknown direction: {d}")
    
    action, speed = action_map[d]
    success = robot_system.send_command(action, speed=speed)
    
    if not success:
        raise RuntimeError(f"Failed to queue command: {action}")
    
    return f"Command sent: {action}"

@app.route('/')
def index():
    return render_template('index.html')

@app.route('/move', methods=['POST'])
def move():
    # Accept JSON { "direction": "forward" } or form/body text
    direction = None
    if request.is_json:
        direction = request.get_json(silent=True, force=True).get('direction')
    if not direction:
        # try form or path-encoded body
        direction = request.form.get('direction') or request.data.decode().strip()
    if not direction:
        return {'status': 'error', 'message': 'No direction provided'}, 400
    return _handle_move(direction)

@app.route('/move/<direction>', methods=['POST', 'GET'])
def move_with_path(direction):
    return _handle_move(direction)

@app.route('/status', methods=['GET'])
def status():
    """Return robot control system status"""
    try:
        robot_status = robot_system.get_status()
        return {'status': 'ok', 'robot': robot_status}
    except Exception as e:
        app.logger.exception("failed to get robot status")
        return {'status': 'error', 'message': str(e)}, 500

def _handle_move(direction):
    direction = (direction or '').lower()
    app.logger.info("move request: %s", direction)
    try:
        result = _call_robot_move(direction)
        robot_status = robot_system.get_status()
        app.logger.info("move result=%s status=%s", result, robot_status)
        return {'status': 'success', 'direction': direction, 'result': result, 'robot_status': robot_status}
    except Exception as e:
        app.logger.exception("robot move failed")
        return {'status': 'error', 'message': str(e)}, 500

if __name__ == '__main__':
    # Ensure you run this file from the src/ directory so local robot_system.py is importable:
    # cd /workspaces/its-car/web-control/robot-web-control/src && python3 server.py
    
    try:
        app.run(host='0.0.0.0', port=8080)
    finally:
        robot_system.shutdown_robot()
        app.logger.info("Server shutdown complete")