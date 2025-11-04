"""
Real-time Sensor Data Logger
Logs Phidget encoder + RS232 sensor data with web dashboard

This uses a background thread for data collection and Flask for the web UI.
Much more suitable for continuous sensor logging than Streamlit.

Requirements:
    pip install flask phidget22 pyserial pandas plotly
"""

import threading
import time
from datetime import datetime
from collections import deque
import json
import pandas as pd
import serial
from pathlib import Path

from Phidget22.PhidgetException import PhidgetException
from Phidget22.Devices.Encoder import Encoder
from Phidget22.Devices.Log import Log
from Phidget22.LogLevel import LogLevel

from flask import Flask, render_template_string, jsonify

# Configuration
ENCODER_RESOLUTION_UM = 10  # micrometers per pulse
MAX_DATA_POINTS = 1000      # Keep last 1000 readings in memory
LOG_FILE = "sensor_data.csv"
SERIAL_PORT = "COM3"        # Adjust for your RS232 port
SERIAL_BAUD = 9600

# Thread-safe data storage
class SensorDataLogger:
    def __init__(self):
        self.lock = threading.Lock()
        self.running = False
        
        # Data storage
        self.encoder_position = 0
        self.encoder_velocity = 0.0
        self.rs232_data = ""
        self.encoder_attached = False
        self.last_update = time.time()
        
        # History for plotting
        self.timestamps = deque(maxlen=MAX_DATA_POINTS)
        self.positions = deque(maxlen=MAX_DATA_POINTS)
        self.velocities = deque(maxlen=MAX_DATA_POINTS)
        self.rs232_values = deque(maxlen=MAX_DATA_POINTS)
        
        # Logging
        self.log_file = None
        
        # Devices
        self.encoder = None
        self.serial_port = None
        
    def start_logging(self):
        """Initialize CSV log file"""
        self.log_file = open(LOG_FILE, 'a', newline='')
        # Write header if file is empty
        if Path(LOG_FILE).stat().st_size == 0:
            self.log_file.write("timestamp,encoder_position,encoder_velocity_mm_s,length_mm,rs232_data\n")
    
    def log_data_point(self):
        """Write current data to CSV"""
        if self.log_file:
            timestamp = datetime.now().isoformat()
            length_mm = (self.encoder_position * ENCODER_RESOLUTION_UM) / 1000.0
            self.log_file.write(f"{timestamp},{self.encoder_position},{self.encoder_velocity},{length_mm},{self.rs232_data}\n")
            self.log_file.flush()
    
    def update_encoder(self, position, velocity):
        """Thread-safe encoder update"""
        with self.lock:
            self.encoder_position = position
            self.encoder_velocity = velocity
            self.last_update = time.time()
            
            # Add to history
            self.timestamps.append(time.time())
            length_mm = (position * ENCODER_RESOLUTION_UM) / 1000.0
            self.positions.append(length_mm)
            self.velocities.append(velocity)
            
            # Log to file
            self.log_data_point()
    
    def update_rs232(self, data):
        """Thread-safe RS232 update"""
        with self.lock:
            self.rs232_data = data
            self.rs232_values.append(data)
    
    def set_encoder_attached(self, attached):
        with self.lock:
            self.encoder_attached = attached
    
    def get_current_state(self):
        """Get current readings (thread-safe)"""
        with self.lock:
            return {
                'encoder_position': self.encoder_position,
                'encoder_velocity': self.encoder_velocity,
                'length_mm': (self.encoder_position * ENCODER_RESOLUTION_UM) / 1000.0,
                'rs232_data': self.rs232_data,
                'encoder_attached': self.encoder_attached,
                'last_update': self.last_update
            }
    
    def get_history(self):
        """Get historical data for plotting"""
        with self.lock:
            return {
                'timestamps': list(self.timestamps),
                'positions': list(self.positions),
                'velocities': list(self.velocities),
                'rs232_values': list(self.rs232_values)
            }

# Global logger instance
logger = SensorDataLogger()

# Phidget event handlers
def on_encoder_position_change(self, positionChange, timeChange, indexTriggered):
    position = self.getPosition()
    velocity = 0.0
    if timeChange > 0:
        velocity = (positionChange * ENCODER_RESOLUTION_UM) / (timeChange / 1000.0)
    logger.update_encoder(position, velocity)

def on_encoder_attach(self):
    logger.set_encoder_attached(True)
    print("Encoder attached!")

def on_encoder_detach(self):
    logger.set_encoder_attached(False)
    print("Encoder detached!")

def on_encoder_error(self, code, description):
    print(f"Encoder error {code}: {description}")

# Initialize Phidget encoder
def init_encoder():
    try:
        Log.enable(LogLevel.PHIDGET_LOG_INFO, "sensor_logger.log")
        
        encoder = Encoder()
        encoder.setChannel(0)
        encoder.setOnPositionChangeHandler(on_encoder_position_change)
        encoder.setOnAttachHandler(on_encoder_attach)
        encoder.setOnDetachHandler(on_encoder_detach)
        encoder.setOnErrorHandler(on_encoder_error)
        
        encoder.openWaitForAttachment(5000)
        logger.encoder = encoder
        print("Encoder initialized successfully")
        return True
    except PhidgetException as e:
        print(f"Failed to initialize encoder: {e}")
        return False

# RS232 reader thread
def rs232_reader_thread():
    """Background thread to read RS232 data"""
    try:
        ser = serial.Serial(SERIAL_PORT, SERIAL_BAUD, timeout=1)
        logger.serial_port = ser
        print(f"RS232 port {SERIAL_PORT} opened")
        
        while logger.running:
            try:
                if ser.in_waiting > 0:
                    data = ser.readline().decode('utf-8').strip()
                    logger.update_rs232(data)
            except Exception as e:
                print(f"RS232 read error: {e}")
            time.sleep(0.01)
            
    except serial.SerialException as e:
        print(f"Failed to open RS232 port: {e}")

# Flask web application
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
        .metrics { display: grid; grid-template-columns: repeat(auto-fit, minmax(200px, 1fr)); gap: 20px; margin: 20px 0; }
        .metric { background: white; padding: 20px; border-radius: 8px; box-shadow: 0 2px 4px rgba(0,0,0,0.1); }
        .metric-label { font-size: 14px; color: #666; margin-bottom: 5px; }
        .metric-value { font-size: 28px; font-weight: bold; color: #2c3e50; }
        .status { display: inline-block; padding: 5px 15px; border-radius: 20px; font-size: 12px; font-weight: bold; }
        .status-connected { background: #2ecc71; color: white; }
        .status-disconnected { background: #e74c3c; color: white; }
        .chart { background: white; padding: 20px; border-radius: 8px; box-shadow: 0 2px 4px rgba(0,0,0,0.1); margin: 20px 0; }
        .controls { background: white; padding: 20px; border-radius: 8px; margin-bottom: 20px; }
        button { background: #3498db; color: white; border: none; padding: 10px 20px; border-radius: 5px; cursor: pointer; font-size: 14px; }
        button:hover { background: #2980b9; }
    </style>
</head>
<body>
    <div class="container">
        <h1>🔬 Real-time Sensor Data Logger</h1>
        
        <div class="controls">
            <button onclick="resetPosition()">Reset Encoder Position</button>
            <button onclick="downloadData()">Download CSV</button>
            <span id="status" class="status status-disconnected">Disconnected</span>
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
                <div class="metric-label">RS232 Data</div>
                <div class="metric-value" id="rs232">--</div>
            </div>
        </div>
        
        <div class="chart">
            <div id="position-chart"></div>
        </div>
        
        <div class="chart">
            <div id="velocity-chart"></div>
        </div>
    </div>
    
    <script>
        // Initialize charts
        Plotly.newPlot('position-chart', [{
            y: [],
            type: 'scatter',
            mode: 'lines',
            name: 'Position (mm)',
            line: {color: '#3498db'}
        }], {
            title: 'Position History',
            xaxis: {title: 'Sample'},
            yaxis: {title: 'Position (mm)'}
        });
        
        Plotly.newPlot('velocity-chart', [{
            y: [],
            type: 'scatter',
            mode: 'lines',
            name: 'Velocity (mm/s)',
            line: {color: '#e74c3c'}
        }], {
            title: 'Velocity History',
            xaxis: {title: 'Sample'},
            yaxis: {title: 'Velocity (mm/s)'}
        });
        
        // Update data every 100ms
        setInterval(updateData, 100);
        
        function updateData() {
            fetch('/data')
                .then(response => response.json())
                .then(data => {
                    // Update metrics
                    document.getElementById('position').textContent = data.encoder_position.toLocaleString();
                    document.getElementById('length').textContent = data.length_mm.toFixed(3);
                    document.getElementById('velocity').textContent = data.encoder_velocity.toFixed(2);
                    document.getElementById('rs232').textContent = data.rs232_data || '--';
                    
                    // Update status
                    const status = document.getElementById('status');
                    if (data.encoder_attached) {
                        status.textContent = 'Connected';
                        status.className = 'status status-connected';
                    } else {
                        status.textContent = 'Disconnected';
                        status.className = 'status status-disconnected';
                    }
                });
            
            // Update charts
            fetch('/history')
                .then(response => response.json())
                .then(data => {
                    Plotly.update('position-chart', {y: [data.positions]}, {}, [0]);
                    Plotly.update('velocity-chart', {y: [data.velocities]}, {}, [0]);
                });
        }
        
        function resetPosition() {
            fetch('/reset', {method: 'POST'})
                .then(response => response.json())
                .then(data => alert(data.message));
        }
        
        function downloadData() {
            window.location.href = '/download';
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

@app.route('/download')
def download_csv():
    from flask import send_file
    return send_file(LOG_FILE, as_attachment=True)

def main():
    print("Starting Sensor Data Logger...")
    
    # Start logging
    logger.start_logging()
    logger.running = True
    
    # Initialize encoder
    if not init_encoder():
        print("Warning: Encoder not initialized. Continuing without encoder.")
    
    # Start RS232 reader thread
    rs232_thread = threading.Thread(target=rs232_reader_thread, daemon=True)
    rs232_thread.start()
    
    # Start web server
    print("\n" + "="*50)
    print("Web dashboard available at: http://localhost:5000")
    print("="*50 + "\n")
    
    try:
        app.run(host='0.0.0.0', port=5000, debug=False)
    except KeyboardInterrupt:
        print("\nShutting down...")
    finally:
        logger.running = False
        if logger.encoder:
            logger.encoder.close()
        if logger.serial_port:
            logger.serial_port.close()
        if logger.log_file:
            logger.log_file.close()

if __name__ == '__main__':
    main()