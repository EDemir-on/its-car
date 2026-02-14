# Robot Web Control

This project provides a web interface to control a robot using HTTP requests. The robot can be controlled using arrow key symbols displayed on a web page. 

## Project Structure

```
robot-web-control
├── src
│   ├── server.py          # Main server script for handling HTTP requests
│   ├── robot.py           # Logic for controlling the robot
│   ├── templates
│   │   └── index.html     # HTML template for the web interface
│   └── static
│       ├── js
│       │   └── main.js     # JavaScript for capturing key events
│       └── css
│           └── style.css    # CSS styles for the web interface
├── requirements.txt        # Python dependencies
├── .gitignore              # Files to ignore in Git
├── Dockerfile              # Instructions for building a Docker image
├── devcontainer.json       # Development container settings
└── README.md               # Project documentation
```

## Setup Instructions

1. **Clone the repository:**
   ```
   git clone <repository-url>
   cd robot-web-control
   ```

2. **Install dependencies:**
   It is recommended to use a virtual environment. You can create one using:
   ```
   python3 -m venv venv
   source venv/bin/activate
   ```
   Then install the required packages:
   ```
   pip install -r requirements.txt
   ```

3. **Run the server:**
   Start the Flask server by running:
   ```
   python src/server.py
   ```

4. **Access the web interface:**
   Open your web browser and navigate to `http://localhost:5000` to access the control interface.

## Usage

- Use the arrow keys on your keyboard to control the robot:
  - Up Arrow: Move Forward
  - Down Arrow: Move Backward
  - Left Arrow: Turn Left
  - Right Arrow: Turn Right

## License

This project is licensed under the MIT License.