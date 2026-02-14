from flask import Flask, render_template, request
import importlib
import logging
import sys

import robot  # Assuming robot.py contains the robot control logic

app = Flask(__name__)
app.logger.setLevel(logging.INFO)

# Log which robot module was loaded (helps debug wrong import / PYTHONPATH)
app.logger.info("robot module: %s", getattr(robot, '__file__', 'unknown'))
# Try reload to ensure latest file is used (useful during development)
try:
    importlib.reload(robot)
    app.logger.info("reloaded robot module: %s", getattr(robot, '__file__', 'unknown'))
except Exception:
    app.logger.exception("failed to reload robot module")

def _call_robot_move(direction):
    """Call robot.move if present, otherwise fallback to direct functions."""
    d = (direction or "").strip().lower()
    # Prefer high-level API if available
    if hasattr(robot, 'move'):
        return robot.move(d)
    # Fallback mapping to low-level functions (robot.move_forward, etc.)
    app.logger.info("robot.move not found, using fallback mapping for: %s", d)
    if d in ('w', 'forward'):
        return getattr(robot, 'move_forward')(0.6)
    if d in ('s', 'back', 'reverse'):
        return getattr(robot, 'move_backward')(0.6)
    if d in ('a', 'left'):
        return getattr(robot, 'turn_left')(0.6)
    if d in ('d', 'right'):
        return getattr(robot, 'turn_right')(0.6)
    if d in ('stop', 'space', 'brake'):
        return getattr(robot, 'stop')()
    raise AttributeError("no matching robot function for: " + d)

def _handle_move(direction):
    direction = (direction or '').lower()
    app.logger.info("move request: %s", direction)
    try:
        result = _call_robot_move(direction)
        return {'status': 'success', 'direction': direction, 'result': result}
    except Exception as e:
        app.logger.exception("robot move failed")
        return {'status': 'error', 'message': str(e)}, 500

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

if __name__ == '__main__':
    # Ensure you run this file from the src/ directory so local robot.py is importable:
    # cd /workspaces/its-car/web-control/robot-web-control/src && python3 server.py
    app.run(host='0.0.0.0', port=8080)