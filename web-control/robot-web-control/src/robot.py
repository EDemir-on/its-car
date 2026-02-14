from gpiozero import PWMLED, DigitalOutputDevice
import threading
import queue
import logging
import time

log = logging.getLogger("robot")
log.setLevel(logging.INFO)

# Command queue and worker
_cmd_q = queue.Queue(maxsize=32)
_current_cmd = None
_worker_thread = None
_worker_stop = threading.Event()

# default PWM for high-level commands
DEFAULT_SPEED = 0.6

# ----------------------------
# Motor pins
# ----------------------------
motorA_in1 = DigitalOutputDevice(17)
motorA_in2 = DigitalOutputDevice(27)
motorB_in3 = DigitalOutputDevice(22)
motorB_in4 = DigitalOutputDevice(23)

motorA_pwm = PWMLED(12)  # ENA
motorB_pwm = PWMLED(13)  # ENB

def _worker():
    global _current_cmd
    log.info("robot worker started")
    while not _worker_stop.is_set():
        try:
            # wait for next command, timeout so we can check stop flag
            cmd = _cmd_q.get(timeout=0.2)
        except queue.Empty:
            continue
        _current_cmd = cmd
        try:
            log.info("EXECUTE COMMAND: %s", cmd)
            # call the appropriate motor helper (these are defined later in this file)
            if cmd == 'forward':
                move_forward(DEFAULT_SPEED)
            elif cmd in ('back', 'backward', 'reverse'):
                move_backward(DEFAULT_SPEED)
            elif cmd == 'left':
                turn_left(DEFAULT_SPEED)
            elif cmd == 'right':
                turn_right(DEFAULT_SPEED)
            elif cmd == 'stop':
                stop()
            else:
                log.warning("unknown command in worker: %s", cmd)
            # do not auto-stop here — leave motors in commanded state until next command
        except Exception:
            log.exception("error executing command: %s", cmd)
        finally:
            _cmd_q.task_done()
    log.info("robot worker stopped")

def move(direction: str):
    """Queue a direction command. direction is a short string (eg 'forward','back','left','right','stop')."""
    if direction is None:
        raise ValueError("direction must be provided")
    d = str(direction).strip().lower()
    # normalize common inputs
    mapping = {
        'w': 'forward', 'forward': 'forward',
        's': 'back', 'back': 'back', 'reverse': 'back',
        'a': 'left', 'left': 'left',
        'd': 'right', 'right': 'right',
        'stop': 'stop', 'space': 'stop', 'brake': 'stop'
    }
    cmd = mapping.get(d, d)
    try:
        # keep only latest commands if queue is full
        _cmd_q.put_nowait(cmd)
    except queue.Full:
        try:
            # drop oldest then enqueue
            _cmd_q.get_nowait()
            _cmd_q.put_nowait(cmd)
        except Exception:
            # as a fallback, block briefly
            _cmd_q.put(cmd, timeout=0.5)
    log.info("queued command: %s", cmd)
    return {'status': 'queued', 'command': cmd}

def get_status():
    """Return simple status info."""
    return {'current': _current_cmd, 'queued': _cmd_q.qsize()}

def shutdown():
    """Stop background worker cleanly."""
    _worker_stop.set()
    if _worker_thread is not None:
        _worker_thread.join(timeout=1.0)

# start worker on import
def _ensure_worker():
    global _worker_thread
    if _worker_thread is None or not _worker_thread.is_alive():
        _worker_stop.clear()
        _worker_thread = threading.Thread(target=_worker, name="robot-worker", daemon=True)
        _worker_thread.start()

_ensure_worker()

# If running under interactive reload, ensure clean shutdown at exit
import atexit
atexit.register(shutdown)

def move_forward(speed):
    motorA_in1.on()
    motorA_in2.off()
    motorB_in3.on()
    motorB_in4.off()
    motorA_pwm.value = speed
    motorB_pwm.value = speed

def move_backward(speed):
    motorA_in1.off()
    motorA_in2.on()
    motorB_in3.off()
    motorB_in4.on()
    motorA_pwm.value = speed
    motorB_pwm.value = speed

def turn_left(speed):
    motorA_in1.off()
    motorA_in2.on()
    motorB_in3.on()
    motorB_in4.off()
    motorA_pwm.value = speed
    motorB_pwm.value = speed

def turn_right(speed):
    motorA_in1.on()
    motorA_in2.off()
    motorB_in3.off()
    motorB_in4.on()
    motorA_pwm.value = speed
    motorB_pwm.value = speed

def stop():
    motorA_pwm.value = 0
    motorB_pwm.value = 0
    motorA_in1.off()
    motorA_in2.off()
    motorB_in3.off()
    motorB_in4.off()