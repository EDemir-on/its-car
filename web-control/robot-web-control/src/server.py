from flask import Flask, render_template, request
import robot  # Assuming robot.py contains the robot control logic
import logging

app = Flask(__name__)
app.logger.setLevel(logging.INFO)

@app.route('/')
def index():
    return render_template('index.html')

def _handle_move(direction):
    direction = (direction or '').lower()
    app.logger.info("move request: %s", direction)
    try:
        robot.move(direction)  # Call the move function from robot.py
        return {'status': 'success', 'direction': direction}
    except Exception as e:
        app.logger.exception("robot.move failed")
        return {'status': 'error', 'message': str(e)}, 500

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

# Accept POST /move/forward (and same by GET for quick testing if desired)
@app.route('/move/<direction>', methods=['POST', 'GET'])
def move_with_path(direction):
    return _handle_move(direction)

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=8080)