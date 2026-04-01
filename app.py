"""
Real-time Sensor Data Logger (Flask)
Logs Phidget encoder + RS232 sensor data with web dashboard.
Adds active RS232 polling (MA,0\\r), start/stop recording, and custom filename.

Requirements:
    pip install flask phidget22 pyserial pandas plotly
"""

import threading
import time
from datetime import datetime
from collections import deque
from typing import List, Optional
import json
import pandas as pd
import serial
from pathlib import Path

from Phidget22.PhidgetException import PhidgetException
from Phidget22.Devices.Encoder import Encoder
from Phidget22.Devices.DCMotor import DCMotor
from Phidget22.Devices.Log import Log
from Phidget22.LogLevel import LogLevel

from flask import Flask, render_template_string, jsonify, request, send_file

import platform

# -----------------------------
# Configuration
# -----------------------------
ENCODER_RESOLUTION_UM = 10    # micrometers per pulse
MAX_DATA_POINTS = 2000        # Keep last N readings in memory
DEFAULT_LOG_DIR = Path("logs") # NEW: folder to keep recordings
DEFAULT_BASE_FILENAME = "sensor_data"  # NEW      

if platform.system() == "Windows":
    SERIAL_PORT = "COM7" # Adjust for your RS232 port
else:
    # Default to Linux (and Mac)
    SERIAL_PORT = "/dev/ttyUSB0"

SERIAL_BAUD = 115200
RS232_COMMAND = "MA,0\r"      # NEW: command to request "ALL outputs"
RS232_POLL_PERIOD_S = 0.0005     # NEW: every 500 ms as in your PowerShell loop
RS232_POST_COMMAND_DELAY_S = 0.0005  # NEW: give device ~100 ms to reply

LOG_SAMPLING_HZ = 10          # NEW: logging rate while recording (10 Hz)

MOTOR_SPEED = 0.1 # Default duty cycle of the motor
MOTOR_DIR = -1 # Flip to make the motor go reverse

# Motor control conf
MOTOR_CONTROL_CYCLE = 0.1   # motor control period, in seconds
MOTOR_SPEED_TARGET = 0.9    # How much encoder value jump on average per encoder cycle (1ms)
MOTOR_VOLTAGE_OFFSET = 0.05 # Voltage tested to almost make it move

# PID
P_COEFF = 0.01
I_COEFF = 0.01
D_COEFF = 0.01

I_LIMIT = 3

ENCODER_PROTECTION_LIMIT = 3000 # Far limit for motor for tripping the protection
ENCODER_SCAN_POSITION = 3000 # Where the encoder should be at when finishing a round of scan

ACCEL_EMA_ALPHA = 0.5 # acceleration moving average smoother

# -----------------------------
FORWARD = 1 # away from encoder
REVERSE = -1 # to the encoder
STOP = 0

# Thread-safe data storage
class SensorDataLogger:
    def __init__(self):
        self.lock = threading.Lock()
        self.running = False
        self.recording = False             # NEW
        self.current_log_path: Optional[Path] = None  # NEW
        self.log_file = None

        # Data storage
        self.encoder_position = 0
        self.encoder_velocity_mm_s = 0.0   # CHANGED: store in mm/s
        self.rs232_raw = ""                # NEW: keep raw line
        self.rs232_outs: List[Optional[float]] = [None, None, None]  # NEW: OUT1..3
        self.encoder_attached = False
        self.last_update = time.time()
        self.position_change = 0

        # History for plotting
        self.timestamps = deque(maxlen=MAX_DATA_POINTS)
        self.positions = deque(maxlen=MAX_DATA_POINTS)
        self.velocities = deque(maxlen=MAX_DATA_POINTS)
        self.rs232_values = deque(maxlen=MAX_DATA_POINTS)  # keep raw or e.g., OUT1
        self.position_change_que = deque(maxlen=MAX_DATA_POINTS)

        # need acceleration as well
        self.encoder_acceleration_mm_s_2 = 0.0
        self.accelerations = deque(maxlen=MAX_DATA_POINTS)

        # Devices
        self.encoder = None
        self.motor = None
        self.motor_dir = STOP
        self.serial_port: Optional[serial.Serial] = None

        # Stored thread
        self.motor_control_thread = None
        self.motor_control_stop_event = threading.Event()

    # ---------- Recording ----------
    def start_recording(self, base_filename: str):
        with self.lock:
            DEFAULT_LOG_DIR.mkdir(parents=True, exist_ok=True)
            timestr = datetime.now().strftime("%Y%m%d_%H%M%S")
            safe_base = base_filename.strip() or DEFAULT_BASE_FILENAME
            fname = f"{safe_base}_{timestr}.csv"
            self.current_log_path = DEFAULT_LOG_DIR / fname
            self.log_file = open(self.current_log_path, 'a', newline='')
            # CSV header
            self.log_file.write(
                "timestamp,encoder_position,encoder_velocity_mm_s,position_change_que,encoder_acceleration_mm_s_2,length_mm,"
                "rs232_out1,rs232_out2,rs232_out3,rs232_raw\n"
            )
            self.log_file.flush()
            self.recording = True

    def stop_recording(self):
        with self.lock:
            self.recording = False
            if self.log_file:
                self.log_file.close()
                self.log_file = None

    def get_record_status(self):
        with self.lock:
            return {
                "recording": self.recording,
                "file": str(self.current_log_path) if self.current_log_path else ""
            }

    # ---------- Logging ----------
    def log_row(self):
        """Write current snapshot to CSV (if recording)."""
        with self.lock:
            if not self.recording or not self.log_file:
                return
            timestamp = datetime.now().isoformat()
            length_mm = (self.encoder_position * ENCODER_RESOLUTION_UM) / 1000.0
            out1 = "" if self.rs232_outs[0] is None else self.rs232_outs[0]
            out2 = "" if self.rs232_outs[1] is None else self.rs232_outs[1]
            out3 = "" if self.rs232_outs[2] is None else self.rs232_outs[2]
            self.log_file.write(
                f"{timestamp},{self.encoder_position},{self.encoder_velocity_mm_s:.6f},{self.position_change},{self.encoder_acceleration_mm_s_2:.6f},{length_mm:.6f},"
                f"{out1},{out2},{out3},{self.rs232_raw}\n"
            )
            self.log_file.flush()

    # ---------- Updates ----------
    def update_encoder(self, position, velocity_mm_s: float,positionChange, time_change: float):
        """Thread-safe encoder update"""
        with self.lock:
            self.encoder_position = position
            self.encoder_velocity_mm_s = velocity_mm_s
            self.position_change = positionChange
            self.last_update = time.time()

            # Calculate acceleration
            if self.velocities and time_change > 0:
                raw_accel = (velocity_mm_s-self.velocities[-1])/(time_change/1000)
                self.encoder_acceleration_mm_s_2 = (
                    ACCEL_EMA_ALPHA * raw_accel +
                    (1 - ACCEL_EMA_ALPHA) * self.encoder_acceleration_mm_s_2
                )
            else:
                self.encoder_acceleration_mm_s_2 = 0

            # Add to history
            self.timestamps.append(time.time())
            length_mm = (position * ENCODER_RESOLUTION_UM) / 1000.0
            self.positions.append(length_mm)
            self.velocities.append(velocity_mm_s)
            self.position_change_que.append(self.position_change)

            # Add acceleration to history
            self.accelerations.append(self.encoder_acceleration_mm_s_2)

    def update_rs232(self, raw_line: str, outs: List[Optional[float]]):
        """Thread-safe RS232 update"""
        with self.lock:
            self.rs232_raw = raw_line
            self.rs232_outs = outs
            # For history/plotting, keep OUT1 as representative (or raw if preferred)
            self.rs232_values.append(outs[0] if outs and outs[0] is not None else raw_line)

    def set_encoder_attached(self, attached):
        with self.lock:
            self.encoder_attached = attached

    # ---------- Read (for API) ----------
    def get_current_state(self):
        with self.lock:
            return {
                'encoder_position': self.encoder_position,
                'encoder_velocity': self.encoder_velocity_mm_s,  # mm/s
                'length_mm': (self.encoder_position * ENCODER_RESOLUTION_UM) / 1000.0,
                'rs232_data': self.rs232_raw,
                'rs232_out1': self.rs232_outs[0],
                'rs232_out2': self.rs232_outs[1],
                'rs232_out3': self.rs232_outs[2],
                'encoder_attached': self.encoder_attached,
                'last_update': self.last_update
            }

    def get_history(self):
        with self.lock:
            return {
                'timestamps': list(self.timestamps),
                'positions': list(self.positions),
                'velocities': list(self.velocities),
                'rs232_values': list(self.rs232_values),
                # add acceleration
                'accelerations': list(self.accelerations)
            }


# Global logger instance
logger = SensorDataLogger()


# -----------------------------
# Phidget event handlers
# -----------------------------
def on_encoder_position_change(self, positionChange, timeChange, indexTriggered):
    # timeChange is in milliseconds; velocity in mm/s:
    # mm/s = positionChange * (um/pulse) / timeChange(ms)
    velocity_mm_s = 0.0
    acceleration_mm_s_2 = 0.0
    if timeChange > 0:
        velocity_mm_s = (positionChange * ENCODER_RESOLUTION_UM) / timeChange  # CHANGED (units fixed)
    position = self.getPosition()
    logger.update_encoder(position, velocity_mm_s, positionChange, timeChange)

def on_encoder_attach(self):
    logger.set_encoder_attached(True)
    print("Encoder attached!")

def on_encoder_detach(self):
    logger.set_encoder_attached(False)
    print("Encoder detached!")

def on_encoder_error(self, code, description):
    print(f"Encoder error {code}: {description}")


def init_encoder():
    try:
        Log.enable(LogLevel.PHIDGET_LOG_INFO, "sensor_logger.log")
        encoder = Encoder()

        # --- VINT HUB SPECIFIC ADDRESSING ---
        encoder.setHubPort(0)               # You plugged it into Hub Port 0
        encoder.setIsHubPortDevice(False)   # ENC1001 is a VINT device, not a raw hub port
        encoder.setChannel(0)               # The encoder channel on the ENC1001
        # ------------------------------------

        encoder.setOnPositionChangeHandler(on_encoder_position_change)
        encoder.setOnAttachHandler(on_encoder_attach)
        encoder.setOnDetachHandler(on_encoder_detach)
        encoder.setOnErrorHandler(on_encoder_error)
        encoder.openWaitForAttachment(5000)

        min_interval = encoder.getMinDataInterval()
        print(f"encoder min interval: {min_interval}")
        encoder.setDataInterval(min_interval)

        logger.encoder = encoder
        print("Encoder initialized successfully")
        return True
    except PhidgetException as e:
        print(f"Failed to initialize encoder: {e}")
        return False

# -----------------------------
# Phidget motor
# -----------------------------

def init_motor():
    try:
        motor = DCMotor()
        motor.openWaitForAttachment(5000)
        min_interval = motor.getMinDataInterval()
        print(f"motor min_interval: {min_interval}")
        motor.setDataInterval(min_interval)
        logger.motor = motor
        print("Motor initialized successfully")
        return True
    except PhidgetException as e:
        print(f"Failed to initialize motor: {e}")
        return False

def set_motor_speed(speed):
    speed_dir = (speed > 0) - (speed < 0)
    speed_real = speed*MOTOR_DIR
    try:
        logger.motor.setTargetVelocity(speed_real)
        logger.motor_dir = speed_dir
        # logging
        with open('motor_log.txt', 'a') as file:
            print(f"{time.time()},{speed_real}", file=file)
        return True
    except Exception as e:
        print('Motor set speed failed')
        return False

def motor_protection_thread():
    while logger.running:
        if logger.motor:
            with logger.lock:
                pos = logger.encoder_position
                direction = logger.motor_dir
            if (pos > ENCODER_PROTECTION_LIMIT and direction == FORWARD) or (pos <= 0 and direction == REVERSE):
                print('Motor stop due to hitting limit')
                try:
                    if logger.motor_control_thread:
                        logger.motor_control_stop_event.set()
                        logger.motor_control_thread.join()
                    set_motor_speed(0)
                    print('Motor stop successfully')
                except Exception as e:
                    print('Motor stop failed')
        time.sleep(0.1)

def motor_speed_control_thread(direction,stop_event):
    last_time = time.time()
    time.sleep(0.1)
    i_sum = 0
    last_e = 0
    while logger.running and not stop_event.is_set():
        current_time = time.time()
        cnt = 0
        total_pos_change = 0
        # iterate over, find all data between two cycles, find speed
        with logger.lock:
            for i in range(len(logger.timestamps)-1, -1, -1):
                if last_time < logger.timestamps[i]:
                    cnt+=1
                    total_pos_change += logger.position_change_que[i]
                else:
                    break
        # get speed
        if cnt>0:
            speed = total_pos_change/cnt
        else:
            speed = 0.0
        # now we have speed, do a PID
        e = direction * MOTOR_SPEED_TARGET - speed
        p = P_COEFF * e
        i = I_COEFF * i_sum
        d = D_COEFF * (e-last_e)
        offset = direction*MOTOR_VOLTAGE_OFFSET

        voltage = p+i+d+offset

        # safety bounding
        voltage = max(-1.0, min(1.0, voltage))
        set_motor_speed(voltage)

        # post process
        last_e = e
        i_sum += e
        if i_sum > I_LIMIT:
            i_sum = I_LIMIT
        elif i_sum < -I_LIMIT:
            i_sum = -I_LIMIT

        # logging
        with open('PID_log.csv', 'a') as file:
            print(f"{current_time},{p},{i},{d},{offset},{voltage},{speed},{e},", file=file)

        last_time = current_time
        stop_event.wait(MOTOR_CONTROL_CYCLE)
    
    print('stopping thread')
    set_motor_speed(0)

# -----------------------------
# RS232 threads
# -----------------------------
def parse_ma_line(line: str) -> List[Optional[float]]:

    """
    Parse a line like: "MA,12.3,45.6,78.9"
    Returns [out1, out2, out3]; gracefully handles missing or non-numeric fields.
    """
    line = line.strip()
    outs: List[Optional[float]] = [None, None, None]
    if not line:
        return outs
    if line.startswith("MA,"):
        payload = line[3:]
    else:
        payload = line
    parts = [p.strip() for p in payload.split(",") if p.strip() != ""]
    for i in range(min(3, len(parts))):
        try:
            outs[i] = float(parts[i])
        except ValueError:
            outs[i] = None
    return outs

def rs232_polling_thread():
    """
    Background thread to actively poll the RS232 device:
    - Write command (MA,0\\r)
    - Small delay
    - Read what's available and parse
    """
    try:
        ser = serial.Serial(
            SERIAL_PORT,
            SERIAL_BAUD,
            timeout=0.2,          # non-blocking-ish
            write_timeout=0.2
        )
        logger.serial_port = ser
        # Flush buffers
        ser.reset_input_buffer()
        ser.reset_output_buffer()
        print(f"RS232 port {SERIAL_PORT} opened")

        while logger.running:
            try:
                if not ser.is_open:
                    ser.open()

                # Write the MA,0 command
                ser.write(RS232_COMMAND.encode('utf-8', errors='ignore'))
                ser.flush()

                # Give device a moment to respond
                time.sleep(RS232_POST_COMMAND_DELAY_S)

                # Read what arrived; try to get a full line
                # Many devices terminate with \\r or \\r\\n
                line = ser.read_until(b'\r')  # stops at \\r or timeout
                if not line:
                    # fallback: anything waiting
                    if ser.in_waiting > 0:
                        line = ser.read(ser.in_waiting)
                raw = line.decode('utf-8', errors='ignore').strip()
                if raw:
                    outs = parse_ma_line(raw)
                    logger.update_rs232(raw, outs)

            except Exception as e:
                print(f"RS232 polling error: {e}")
            finally:
                # Respect polling cadence (approx total: post-delay + sleep)
                time.sleep(max(0.0, RS232_POLL_PERIOD_S - RS232_POST_COMMAND_DELAY_S))

    except serial.SerialException as e:
        print(f"Failed to open RS232 port: {e}")


def logging_thread():
    """Steady-rate logger independent of event callbacks."""
    period = 1.0 / LOG_SAMPLING_HZ
    next_t = time.perf_counter()
    while logger.running:
        now = time.perf_counter()
        if now >= next_t:
            logger.log_row()
            next_t += period
        # small sleep to avoid busy wait
        time.sleep(0.002)


# -----------------------------
# Flask web application
# -----------------------------
app = Flask(__name__)

HTML_TEMPLATE = """
<!DOCTYPE html>
<html>
<head>
    <title>Sensor Data Logger</title>
        <script src="https://cdn.plot.ly/plotly-latest.min.js"></script>
    <style>
     body { font-family: Arial, sans-serif; margin: 20px; background: #f5f5f5; }
        .container { max-width: 1400px; margin: 0 auto; }
        h1 { color: #333; }
        .grid { display: grid; grid-template-columns: 1fr 1fr; gap: 20px; }
        .metrics { display: grid; grid-template-columns: repeat(auto-fit, minmax(220px, 1fr)); gap: 20px; margin: 20px 0; }
        .metric, .panel { background: white; padding: 20px; border-radius: 8px; box-shadow: 0 2px 4px rgba(0,0,0,0.1); }
        .metric-label { font-size: 14px; color: #666; margin-bottom: 5px; }
        .metric-value { font-size: 28px; font-weight: bold; color: #2c3e50; }
        .status { display: inline-block; padding: 5px 15px; border-radius: 20px; font-size: 12px; font-weight: bold; margin-left: 10px;}
        .status-connected { background: #2ecc71; color: white; }
        .status-disconnected { background: #e74c3c; color: white; }
        .recording-on { background: #d35400; color: white; }
        button { background: #3498db; color: white; border: none; padding: 10px 20px; border-radius: 5px; cursor: pointer; font-size: 14px; }
        button:hover { background: #2980b9; }
        input[type="text"] { padding: 10px; border-radius: 5px; border: 1px solid #ccc; width: 260px; margin-right: 10px; }
        .charts .panel { margin: 20px 0; }
        .muted { color: #888; font-size: 12px; }
    </style>
</head>
<body>
    <div class="container">
        <h1>🔬 Real-time Sensor Data Logger
            <span id="encStatus" class="status status-disconnected">Encoder: Disconnected</span>
            <span id="recStatus" class="status">Recording: Off</span>
        </h1>

        <div class="panel">
            <input id="filename" type="text" placeholder="Enter base filename (optional)" />
            <button onclick="startRecording()">Start Recording</button>
            <button onclick="stopRecording()">Stop Recording</button>
            <button onclick="downloadData()">Download Current CSV</button>
            <button onclick="resetEncoderPos()">Reset Encoder Position</button>
            <button onclick="motorForward()">Motor Forward</button>
            <button onclick="motorStop()">Motor Stop</button>
            <button onclick="motorReverse()">Motor Reverse</button>
            <button onclick="motorForwardThread()">Motor Forward Thread</button>
            <button onclick="motorReverseThread()">Motor Reverse Thread</button>
            <span class="muted" id="currentFile"></span>
        </div>

        <div class="metrics">
            <div class="metric">
                <div class="metric-label">Encoder Position</div>
                <div class="metric-value" id="position">0</div>
            </div>
            <div class="metric">
                <div class="metric-label">Length (mm)</div>
                <div class="metric-value" id="length">0.000</div>
            </div>
            <div class="metric">
                <div class="metric-label">Velocity (mm/s)</div>
                <div class="metric-value" id="velocity">0.00</div>
            </div>
            <div class="metric">
                <div class="metric-label">RS232 Raw</div>
                <div class="metric-value" id="rs232">--</div>
            </div>
            <div class="metric">
                <div class="metric-label">OUT1</div>
                <div class="metric-value" id="out1">--</div>
            </div>
            <div class="metric">
                <div class="metric-label">OUT2</div>
                <div class="metric-value" id="out2">--</div>
            </div>
            <div class="metric">
                <div class="metric-label">OUT3</div>
                <div class="metric-value" id="out3">--</div>
            </div>
        </div>

        <div class="charts">
            <div class="panel">
                <div id="position-chart"></div>
            </div>
            <div class="panel">
                <div id="velocity-chart"></div>
            </div>
            <div class="panel">
                <div id="acceleration-chart"></div>
            </div>
        </div>
    </div>

    <script>
        // Charts
        Plotly.newPlot('position-chart', [{
            y: [],
            type: 'scatter', mode: 'lines', name: 'Position (mm)',
            line: {color: '#3498db'}
        }], { title: 'Position History', xaxis: {title: 'Sample'}, yaxis: {title: 'Position (mm)'} });

        Plotly.newPlot('velocity-chart', [{
            y: [],
            type: 'scatter', mode: 'lines', name: 'Velocity (mm/s)',
            line: {color: '#e74c3c'}
        }], { title: 'Velocity History', xaxis: {title: 'Sample'}, yaxis: {title: 'Velocity (mm/s)'} });

        Plotly.newPlot('acceleration-chart', [{
            y: [],
            type: 'scatter', mode: 'lines', name: 'Acceleration (mm/s^2)',
            line: {color: '#e74c3c'}
        }], { title: 'Acceleration History', xaxis: {title: 'Sample'}, yaxis: {title: 'Acceleration (mm/s^2)'} });

        async function refresh() {
            try {
                const [dataRes, histRes, recRes] = await Promise.all([
                    fetch('/data'), fetch('/history'), fetch('/record/status')
                ]);
                const data = await dataRes.json();
                const hist = await histRes.json();
                const rec = await recRes.json();

                // Metrics
                document.getElementById('position').textContent = data.encoder_position.toLocaleString();
                document.getElementById('length').textContent = data.length_mm.toFixed(3);
                document.getElementById('velocity').textContent = data.encoder_velocity.toFixed(2);
                document.getElementById('rs232').textContent = data.rs232_data || '--';
                document.getElementById('out1').textContent = (data.rs232_out1 ?? '--');
                document.getElementById('out2').textContent = (data.rs232_out2 ?? '--');
                document.getElementById('out3').textContent = (data.rs232_out3 ?? '--');

                // Encoder status
                const enc = document.getElementById('encStatus');
                if (data.encoder_attached) {
                    enc.textContent = 'Encoder: Connected';
                    enc.className = 'status status-connected';
                } else {
                    enc.textContent = 'Encoder: Disconnected';
                    enc.className = 'status status-disconnected';
                }

                // Recording status
                const recStatus = document.getElementById('recStatus');
                if (rec.recording) {
                    recStatus.textContent = 'Recording: On';
                    recStatus.className = 'status recording-on';
                    document.getElementById('currentFile').textContent = rec.file ? ('→ ' + rec.file) : '';
                } else {
                    recStatus.textContent = 'Recording: Off';
                    recStatus.className = 'status';
                    document.getElementById('currentFile').textContent = '';
                }

                // Charts
                Plotly.update('position-chart', {y: [hist.positions]}, {}, [0]);
                Plotly.update('velocity-chart', {y: [hist.velocities]}, {}, [0]);
                // chart for acceleration
                Plotly.update('acceleration-chart', {y: [hist.accelerations]}, {}, [0]);
            } catch (e) {
                console.error(e);
            }
        }

        setInterval(refresh, 250);

        async function startRecording() {
            const fname = document.getElementById('filename').value || '';
            const res = await fetch('/record/start', {
                method: 'POST',
                headers: {'Content-Type': 'application/json'},
                body: JSON.stringify({ filename: fname })
            });
            const msg = await res.json();
            alert(msg.message || 'Started');
        }

        async function stopRecording() {
            const res = await fetch('/record/stop', { method: 'POST' });
            const msg = await res.json();
            alert(msg.message || 'Stopped');
        }

        function downloadData() {
            window.location.href = '/download';
        }

        // encoder reset
        async function resetEncoderPos() {
            try {
                const res = await fetch('/reset', { method: 'POST' });
                const msg = await res.json();
                console.log(msg.message);
            } catch (e) {
                alert("Failed to reset position. Is the server running?");
                console.error(e);
            }
        }

        // Motor testing function
        async function motorForward() {
            try {
                const res = await fetch('/motor_forward', { method: 'POST' });
                const msg = await res.json();
                console.log(msg.message);
            } catch (e) {
                alert("Failed to control motor. Is the server running?");
                console.error(e);
            }
        }

        async function motorReverse() {
            try {
                const res = await fetch('/motor_reverse', { method: 'POST' });
                const msg = await res.json();
                console.log(msg.message);
            } catch (e) {
                alert("Failed to control motor. Is the server running?");
                console.error(e);
            }
        }

        async function motorStop() {
            try {
                const res = await fetch('/motor_stop', { method: 'POST' });
                const msg = await res.json();
                console.log(msg.message);
            } catch (e) {
                alert("Failed to control motor. Is the server running?");
                console.error(e);
            }
        }

        async function motorForwardThread() {
            try {
                const res = await fetch('/motor_forward_thread', { method: 'POST' });
                const msg = await res.json();
                console.log(msg.message);
            } catch (e) {
                alert("Failed to control motor. Is the server running?");
                console.error(e);
            }
        }

        async function motorReverseThread() {
            try {
                const res = await fetch('/motor_reverse_thread', { method: 'POST' });
                const msg = await res.json();
                console.log(msg.message);
            } catch (e) {
                alert("Failed to control motor. Is the server running?");
                console.error(e);
            }
        }

    </script>
</body>
</html>
"""

@app.route('/')
def index():
    return render_template_string(HTML_TEMPLATE)

@app.route('/data')
def get_data():
    return jsonify(logger.get_current_state())

@app.route('/history')
def get_history():
    return jsonify(logger.get_history())

@app.route('/reset', methods=['POST'])
def reset_position():
    if logger.encoder:
        try:
            logger.encoder.setPosition(0)
            return jsonify({'message': 'Position reset successfully'})
        except Exception as e:
            return jsonify({'message': f'Reset failed: {e}'}), 500
    return jsonify({'message': 'Encoder not connected'}), 400

# ----- Recording endpoints -----
@app.route('/record/start', methods=['POST'])
def record_start():
    data = request.get_json(silent=True) or {}
    base_filename = data.get('filename', '').strip()
    logger.start_recording(base_filename)
    return jsonify({'message': f"Recording started: {logger.current_log_path.name}"})

@app.route('/record/stop', methods=['POST'])
def record_stop():
    prev = logger.current_log_path.name if logger.current_log_path else ''
    logger.stop_recording()
    return jsonify({'message': f"Recording stopped. File: {prev}"})

@app.route('/record/status')
def record_status():
    return jsonify(logger.get_record_status())

@app.route('/download')
def download_csv():
    # If currently recording, return that file; else try last known file or default path
    path: Optional[Path] = logger.current_log_path
    if path and path.exists():
        return send_file(str(path), as_attachment=True)
    # fallback to default legacy file if exists
    legacy = Path("sensor_data.csv")
    if legacy.exists():
        return send_file(str(legacy), as_attachment=True)
    return jsonify({"message": "No recording file available yet."}), 404

# Motor Testing Functions
@app.route('/motor_stop', methods=['POST'])
def motor_stop():
    if logger.motor:
        old_motor_dir = logger.motor_dir
        try:
            if logger.motor_control_thread and logger.motor_control_thread.is_alive():
                logger.motor_control_stop_event.set()
                logger.motor_control_thread.join()
            set_motor_speed(0)
            return jsonify({'message': 'Motor stop successfully'})
        except Exception as e:
            logger.motor_dir = old_motor_dir
            return jsonify({'message': f'Motor stop failed: {e}'}), 500
    return jsonify({'message': 'Motor not connected'}), 400

@app.route('/motor_forward', methods=['POST'])
def motor_forward():
    if logger.motor:
        old_motor_dir = logger.motor_dir
        try:
            set_motor_speed(MOTOR_SPEED)
            return jsonify({'message': 'Motor run forward successfully'})
        except Exception as e:
            logger.motor_dir = old_motor_dir
            return jsonify({'message': f'Motor run forward failed: {e}'}), 500
    return jsonify({'message': 'Motor not connected'}), 400

@app.route('/motor_forward_thread', methods=['POST'])
def motor_forward_thread():
    if logger.motor:
        try:
            if logger.motor_control_thread:
                logger.motor_control_stop_event.set()
                logger.motor_control_thread.join()
            logger.motor_control_stop_event = threading.Event()
            logger.motor_control_thread = threading.Thread(target=motor_speed_control_thread, args=(FORWARD,logger.motor_control_stop_event))
            logger.motor_control_thread.start()
            return jsonify({'message': 'Motor run forward successfully'})
        except Exception as e:
            return jsonify({'message': f'Motor run forward failed: {e}'}), 500
    return jsonify({'message': 'Thread started'}), 400

@app.route('/motor_reverse', methods=['POST'])
def motor_reverse():
    if logger.motor:
        old_motor_dir = logger.motor_dir
        try:
            set_motor_speed(-1*MOTOR_SPEED)
            return jsonify({'message': 'Motor reverse successfully'})
        except Exception as e:
            logger.motor_dir = old_motor_dir
            return jsonify({'message': f'Motor reverse failed: {e}'}), 500
    return jsonify({'message': 'Motor not connected'}), 400

@app.route('/motor_reverse_thread', methods=['POST'])
def motor_reverse_thread():
    if logger.motor:
        old_motor_dir = logger.motor_dir
        try:
            if logger.motor_control_thread:
                logger.motor_control_stop_event.set()
                logger.motor_control_thread.join()
            logger.motor_control_stop_event = threading.Event()
            logger.motor_control_thread = threading.Thread(target=motor_speed_control_thread, args=(REVERSE,logger.motor_control_stop_event))
            logger.motor_control_thread.start()
            return jsonify({'message': 'Motor reverse successfully'})
        except Exception as e:
            logger.motor_dir = old_motor_dir
            return jsonify({'message': f'Motor reverse failed: {e}'}), 500
    return jsonify({'message': 'Motor not connected'}), 400

def main():
    print("Starting Sensor Data Logger...")

    logger.running = True

    # Initialize encoder (non-fatal if not present)
    if not init_encoder():
        print("Warning: Encoder not initialized. Continuing without encoder.")
    
    if not init_motor():
        print("Warning: Motor not initialized. Continuing without motor.")

    # Start RS232 polling thread (active command-write + read + parse)
    rs232_thread = threading.Thread(target=rs232_polling_thread, daemon=True)
    rs232_thread.start()

    # Start steady logging thread (writes when recording=True)
    log_thread = threading.Thread(target=logging_thread, daemon=True)
    log_thread.start()

    motor_thread = threading.Thread(target=motor_protection_thread,daemon=True)
    motor_thread.start()

    # Start web server
    print("\n" + "="*50)
    print("Web dashboard available at: http://localhost:5000")
    print("="*50 + "\n")

    
    # Hide request logs
    import logging
    logging.getLogger('werkzeug').setLevel(logging.ERROR)


    try:
        app.run(host='0.0.0.0', port=5000, debug=False)
    except KeyboardInterrupt:
        print("\nShutting down...")
    finally:
        logger.running = False
        if logger.encoder:
            logger.encoder.close()
        if logger.motor:
            logger.motor.close()
        if logger.serial_port:
            try:
                logger.serial_port.close()
            except Exception:
                pass
        if logger.log_file:
            logger.log_file.close()

if __name__ == '__main__':
    main()
