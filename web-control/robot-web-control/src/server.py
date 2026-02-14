from flask import Flask, render_template, request
import robot  # Assuming robot.py contains the robot control logic

app = Flask(__name__)

@app.route('/')
def index():
    return render_template('index.html')

@app.route('/move', methods=['POST'])
def move():
    direction = request.json.get('direction')
    if direction:
        robot.move(direction)  # Call the move function from robot.py
        return {'status': 'success', 'direction': direction}
    return {'status': 'error', 'message': 'No direction provided'}, 400

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=8080)