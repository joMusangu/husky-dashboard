"""
Flask API Server for ROS Sensor Data Poisoning Detection

Uses HTTP polling for real-time updates (no WebSocket needed)
"""

# Standard library imports
import os
import threading
from pathlib import Path
import json
import zipfile
import time
import math
from flask import Flask, request, jsonify, send_from_directory
from flask_cors import CORS
import numpy as np

# Local application imports
from backend import (
    ROSBagParser, AnomalyDetector, 
    RealtimeProcessor, SensorReading
)
from poison_injector import (
    PoisonInjector, PoisonConfig, PoisonType,
    get_preset_attack, list_preset_attacks
)

app = Flask(__name__, static_folder='.')

# Configure CORS
CORS(app, resources={r"/*": {"origins": "*"}})

# Poison injection is now enabled for testing and validation

@app.after_request
def after_request(response):
    response.headers.add('Access-Control-Allow-Origin', '*')
    response.headers.add('Access-Control-Allow-Headers', 'Content-Type,Authorization')
    response.headers.add('Access-Control-Allow-Methods', 'GET,POST,PUT,DELETE,OPTIONS')
    return response

# Global state
active_processors = {}
processor_counter = 0
processor_lock = threading.Lock()


@app.route('/')
def index():
    """Serve the main dashboard"""
    return send_from_directory('.', 'index.html')


@app.route('/api/health', methods=['GET'])
def health():
    """Health check endpoint"""
    return jsonify({
        'status': 'healthy',
        'message': 'ROS Sensor Data Poisoning Detection API is running'
    })


@app.route('/api/process/start', methods=['POST', 'OPTIONS'])
def start_processing():
    """Start processing ROS bag or synthetic data"""
    if request.method == 'OPTIONS':
        return '', 200
    
    global processor_counter
    
    data = request.json or {}
    config = data.get('config', {})
    use_synthetic = data.get('useSynthetic', True)
    file_path = data.get('filePath')
    
    print(f"[API] Start processing: synthetic={use_synthetic}, file={file_path}")
    
    with processor_lock:
        processor_id = f"processor_{processor_counter}"
        processor_counter += 1
    
    # Create detector and processor
    prediction_window = config.get('predictionWindow', 5)
    detector = AnomalyDetector(prediction_window=prediction_window)
    processor = RealtimeProcessor(detector)
    
    # Initialize poison injector if configured
    poison_injector = None
    poison_configs = config.get('poisonConfigs', [])
    if poison_configs:
        poison_injector = PoisonInjector()
        for pc in poison_configs:
            # Check if it's a preset
            if 'preset' in pc:
                preset_name = pc['preset']
                preset_configs = get_preset_attack(preset_name)
                for preset_config in preset_configs:
                    poison_injector.add_poison(preset_config)
            else:
                # Convert dict to PoisonConfig
                poison_type = PoisonType(pc['poisonType'])
                poison_config = PoisonConfig(
                    poison_type=poison_type,
                    start_time=pc.get('startTime', 0.0),
                    duration=pc.get('duration', 1.0),
                    intensity=pc.get('intensity', 1.0),
                    target_sensor=pc.get('targetSensor', 'gps'),
                    jump_distance=pc.get('jumpDistance', 50.0),
                    drift_rate=pc.get('driftRate', 5.0),
                    noise_stddev=pc.get('noiseStddev', 0.1),
                    bias_value=pc.get('biasValue', 0.5),
                    scale_factor=pc.get('scaleFactor', 2.0)
                )
                poison_injector.add_poison(poison_config)
    
    with processor_lock:
        active_processors[processor_id] = {
            'processor': processor,
            'detector': detector,
            'poison_injector': poison_injector,
            'readings': [],
            'results': [],
            'total_anomalies': 0,  # Track cumulative anomaly count per run
            'all_anomalies': [],  # Track ALL anomalies separately (not just recent window)
            'injected_poisons': [],  # Track injected poisons for validation
            'config': config,
            'running': True,
            'error': None
        }
    
    processor.start()
    
    # Start processing in background thread
    if use_synthetic or not file_path:
        thread = threading.Thread(
            target=process_synthetic_data,
            args=(processor_id, config)
        )
    else:
        thread = threading.Thread(
            target=process_ros_bag,
            args=(processor_id, file_path, config)
        )
    
    thread.daemon = True
    thread.start()
    
    return jsonify({
        'processorId': processor_id,
        'status': 'started',
        'message': 'Processing started'
    })


def process_synthetic_data(processor_id, config):
    """Generate and process synthetic sensor data"""
    print(f"[PROCESSOR {processor_id}] Starting synthetic data processing")
    
    if processor_id not in active_processors:
        return
    
    proc_data = active_processors[processor_id]
    processor = proc_data['processor']
    
    # Generate synthetic data
    readings = []
    lat, lon = 33.5779, -101.8552
    
    for i in range(200):
        if processor_id not in active_processors or not proc_data['running']:
            break
            
        t = i * 0.1
        lat += 0.00001 * (1 + 0.1 * np.sin(t))
        lon += 0.00001 * (1 + 0.1 * np.cos(t))
        
        reading = SensorReading(
            timestamp=t,
            gps_lat=lat,
            gps_lon=lon,
            velocity=2.0 + 0.5 * np.sin(t)
        )
        
        # Apply poison injection if configured
        if proc_data.get('poison_injector'):
            modified_reading, is_poisoned, poison_info = proc_data['poison_injector'].inject(reading, t)
            if is_poisoned:
                proc_data['injected_poisons'].append({
                    'timestamp': t,
                    'poisons': poison_info['applied_poisons']
                })
            reading = modified_reading
        
        readings.append(reading)
        processor.add_reading(reading)
        proc_data['readings'].append(reading)
        
        # For synthetic data, let the status endpoint drain and count results
        # to keep logic consistent with real bag processing.
        time.sleep(0.05)  # 50ms per reading for visualization
    
    # Wait for processing to complete
    time.sleep(0.5)
    processor.stop()
    proc_data['running'] = False
    
    print(f"[PROCESSOR {processor_id}] Synthetic data processing complete: {len(readings)} readings")


def process_ros_bag(processor_id, bag_path, config):
    """Process actual ROS bag file"""
    print(f"[PROCESSOR {processor_id}] Processing bag file: {bag_path}")
    
    if processor_id not in active_processors:
        return
    
    proc_data = active_processors[processor_id]
    processor = proc_data['processor']
    parser = ROSBagParser()
    
    try:
        # Verify path exists
        if not Path(bag_path).exists():
            error_msg = f"Bag file not found: {bag_path}"
            print(f"[ERROR] {error_msg}")
            proc_data['error'] = error_msg
            processor.stop()
            proc_data['running'] = False
            return
        
        # Parse the bag file
        print(f"[PROCESSOR {processor_id}] Parsing bag file...")
        readings = parser.parse_bag(str(bag_path))
        
        if not readings:
            error_msg = "No sensor readings extracted. Check topic names."
            print(f"[WARNING] {error_msg}")
            proc_data['error'] = error_msg
            processor.stop()
            proc_data['running'] = False
            return
        
        print(f"[PROCESSOR {processor_id}] Parsed {len(readings)} readings, starting processing...")
        
        # Process each reading
        for i, reading in enumerate(readings):
            if processor_id not in active_processors or not proc_data['running']:
                break
            
            # Apply poison injection if configured
            if proc_data.get('poison_injector'):
                modified_reading, is_poisoned, poison_info = proc_data['poison_injector'].inject(reading, reading.timestamp)
                if is_poisoned:
                    proc_data['injected_poisons'].append({
                        'timestamp': reading.timestamp,
                        'poisons': poison_info['applied_poisons']
                    })
                reading = modified_reading
            
            proc_data['readings'].append(reading)
            processor.add_reading(reading)
            
            # Log progress every 100 readings
            if (i + 1) % 100 == 0:
                print(f"[PROCESSOR {processor_id}] Processed {i + 1}/{len(readings)} readings")
            
            time.sleep(0.001)  # 1ms per reading - very fast for demo (40k readings = ~40 sec)
        
        time.sleep(0.5)
        processor.stop()
        proc_data['running'] = False
        
        print(f"[PROCESSOR {processor_id}] Bag processing complete: {len(proc_data['readings'])} readings")
        
    except Exception as e:
        print(f"[ERROR] Error processing bag: {e}")
        import traceback
        traceback.print_exc()
        proc_data['error'] = str(e)
        processor.stop()
        proc_data['running'] = False


@app.route('/api/process/<processor_id>/status', methods=['GET'])
def get_status(processor_id):
    """Get current processing status - polled by frontend for real-time updates"""
    if processor_id not in active_processors:
        return jsonify({'error': 'Processor not found'}), 404
    
    proc_data = active_processors[processor_id]
    processor = proc_data['processor']
    detector = proc_data['detector']
    readings = proc_data['readings']
    results = proc_data['results']
    
    # Get latest results from the processor
    latest_results = processor.get_results()
    results.extend(latest_results)
    
    # Count NEW anomalies and add to cumulative total
    new_anomalies = [r for r in latest_results if r.get('anomaly') is not None]
    proc_data['total_anomalies'] += len(new_anomalies)
    
    # Add new anomalies to the persistent all_anomalies list
    for result in new_anomalies:
        anomaly = result.get('anomaly')
        if anomaly:
            proc_data['all_anomalies'].append({
                'timestamp': anomaly['timestamp'],
                'severity': anomaly['severity'],
                'reasons': anomaly['reasons'],
                'position': anomaly.get('position', {})
            })
    
    # Keep only the most recent 1000 results to avoid UI slowdown
    # (for trajectory display only - anomalies are tracked separately)
    if len(results) > 1000:
        proc_data['results'] = results[-1000:]
        results = proc_data['results']
    
    # Calculate metrics - use cumulative total, not recent window
    total_readings = len(readings)
    total_anomalies = proc_data['total_anomalies']
    
    # Use ALL anomalies (persistent list), not just recent window
    all_anomalies = proc_data['all_anomalies']
    
    # Get trust scores
    trust_scores = detector.trust_manager.get_scores()
    
    # Get latest reading
    latest_reading = readings[-1] if readings else None
    
    # Format sensor data with actual values
    sensor_data = {}
    
    # Add trust scores
    for sensor, score in trust_scores.items():
        status = 'healthy' if score > 0.8 else ('warning' if score > 0.5 else 'critical')
        sensor_data[sensor] = {
            'status': status,
            'trustScore': score,
            'lastReading': latest_reading.timestamp if latest_reading else None
        }
    
    # Add actual sensor readings from latest reading
    if latest_reading:
        # GPS data
        sensor_data['gps'] = {
            **sensor_data.get('gps', {}),
            'lat': latest_reading.gps_lat,
            'lon': latest_reading.gps_lon,
            'alt': latest_reading.gps_altitude
        }
        
        # IMU data - convert quaternion to roll/pitch/yaw
        qx, qy, qz, qw = latest_reading.imu_orientation
        # Roll (x-axis rotation)
        sinr_cosp = 2 * (qw * qx + qy * qz)
        cosr_cosp = 1 - 2 * (qx * qx + qy * qy)
        roll = math.atan2(sinr_cosp, cosr_cosp)
        # Pitch (y-axis rotation)
        sinp = 2 * (qw * qy - qz * qx)
        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi / 2, sinp)
        else:
            pitch = math.asin(sinp)
        # Yaw (z-axis rotation)
        siny_cosp = 2 * (qw * qz + qx * qy)
        cosy_cosp = 1 - 2 * (qy * qy + qz * qz)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        
        sensor_data['imu'] = {
            **sensor_data.get('imu', {}),
            'roll': math.degrees(roll),
            'pitch': math.degrees(pitch),
            'yaw': math.degrees(yaw)
        }
        
        # Odometry data
        sensor_data['odometry'] = {
            **sensor_data.get('odometry', {}),
            'x': latest_reading.wheel_odom_x,
            'y': latest_reading.wheel_odom_y,
            'velocity': latest_reading.velocity
        }
    
    # Format trajectory data
    trajectory_data = []
    for reading in readings:
        is_poisoned = any(
            r.get('anomaly') and abs(r['timestamp'] - reading.timestamp) < 0.1
            for r in results
        )
        trajectory_data.append({
            'lat': reading.gps_lat,
            'lon': reading.gps_lon,
            'timestamp': reading.timestamp,
            'isPoisoned': is_poisoned
        })
    
    # Format anomalies - use ALL anomalies (persistent list)
    formatted_anomalies = all_anomalies.copy()
    
    # Calculate processing time
    avg_processing_time = 0
    if results:
        processing_times = [r.get('processing_time', 0) for r in results]
        avg_processing_time = sum(processing_times) / len(processing_times) * 1000
    
    response = {
        'isProcessing': proc_data['running'],
        'metrics': {
            'totalReadings': total_readings,
            'anomaliesDetected': total_anomalies,  # Use cumulative total, not recent window
            'detectionRate': (total_anomalies / total_readings * 100) if total_readings > 0 else 0,
            'avgProcessingTime': avg_processing_time,
            'currentTimestamp': latest_reading.timestamp if latest_reading else 0
        },
        'sensorData': sensor_data,
        'trajectoryData': trajectory_data,
        'anomalies': formatted_anomalies
    }
    
    # Get poison injection info
    poison_info = None
    if proc_data.get('poison_injector'):
        poison_info = {
            'enabled': True,
            'injectedCount': len(proc_data.get('injected_poisons', [])),
            'stats': proc_data['poison_injector'].get_injection_stats()
        }
    else:
        poison_info = {'enabled': False}
    
    response['poisonInjection'] = poison_info
    
    if proc_data.get('error'):
        response['error'] = proc_data['error']
    
    return jsonify(response)


@app.route('/api/process/<processor_id>/stop', methods=['POST'])
def stop_processing(processor_id):
    """Stop processing"""
    if processor_id not in active_processors:
        return jsonify({'error': 'Processor not found'}), 404
    
    proc_data = active_processors[processor_id]
    processor = proc_data['processor']
    
    proc_data['running'] = False
    processor.stop()
    
    return jsonify({
        'status': 'stopped',
        'message': 'Processing stopped'
    })


@app.route('/api/poison/presets', methods=['GET'])
def get_poison_presets():
    """Get list of available preset poison attack scenarios"""
    presets = list_preset_attacks()
    return jsonify({
        'presets': presets,
        'descriptions': {
            'quick_test': 'Quick GPS jump test attacks',
            'sustained_drift': 'Sustained GPS drift attack',
            'sensor_freeze': 'GPS sensor freeze attack',
            'imu_attack': 'IMU noise and bias attacks',
            'multi_sensor': 'Multi-sensor coordinated attack',
            'intermittent': 'Intermittent GPS jump attacks'
        }
    })


@app.route('/api/poison/types', methods=['GET'])
def get_poison_types():
    """Get list of available poison types"""
    types = [pt.value for pt in PoisonType]
    return jsonify({
        'types': types,
        'descriptions': {
            'gps_jump': 'Sudden GPS position jump',
            'gps_drift': 'Gradual GPS position drift',
            'gps_freeze': 'GPS readings freeze at current position',
            'imu_noise': 'Add noise to IMU readings',
            'imu_bias': 'Add bias to IMU readings',
            'odom_scaling': 'Scale odometry values',
            'velocity_spike': 'Sudden velocity change',
            'altitude_jump': 'Altitude anomaly',
            'sensor_dropout': 'Sensor drops out',
            'replay_attack': 'Replay old sensor data'
        }
    })


@app.route('/api/process/<processor_id>/poison/stats', methods=['GET'])
def get_poison_stats(processor_id):
    """Get poison injection statistics for a processor"""
    if processor_id not in active_processors:
        return jsonify({'error': 'Processor not found'}), 404
    
    proc_data = active_processors[processor_id]
    poison_injector = proc_data.get('poison_injector')
    
    if not poison_injector:
        return jsonify({
            'enabled': False,
            'message': 'Poison injection not enabled for this processor'
        })
    
    stats = poison_injector.get_injection_stats()
    return jsonify({
        'enabled': True,
        'stats': stats,
        'injected_poisons': proc_data.get('injected_poisons', [])
    })


@app.route('/api/poison/presets', methods=['GET'])
def get_poison_presets():
    """Get list of available preset poison attack scenarios"""
    presets = list_preset_attacks()
    return jsonify({
        'presets': presets,
        'descriptions': {
            'quick_test': 'Quick GPS jump test attacks',
            'sustained_drift': 'Sustained GPS drift attack',
            'sensor_freeze': 'GPS sensor freeze attack',
            'imu_attack': 'IMU noise and bias attacks',
            'multi_sensor': 'Multi-sensor coordinated attack',
            'intermittent': 'Intermittent GPS jump attacks'
        }
    })


@app.route('/api/poison/types', methods=['GET'])
def get_poison_types():
    """Get list of available poison types"""
    types = [pt.value for pt in PoisonType]
    return jsonify({
        'types': types,
        'descriptions': {
            'gps_jump': 'Sudden GPS position jump',
            'gps_drift': 'Gradual GPS position drift',
            'gps_freeze': 'GPS readings freeze at current position',
            'imu_noise': 'Add noise to IMU readings',
            'imu_bias': 'Add bias to IMU readings',
            'odom_scaling': 'Scale odometry values',
            'velocity_spike': 'Sudden velocity change',
            'altitude_jump': 'Altitude anomaly',
            'sensor_dropout': 'Sensor drops out',
            'replay_attack': 'Replay old sensor data'
        }
    })


@app.route('/api/process/<processor_id>/poison/stats', methods=['GET'])
def get_poison_stats(processor_id):
    """Get poison injection statistics for a processor"""
    if processor_id not in active_processors:
        return jsonify({'error': 'Processor not found'}), 404
    
    proc_data = active_processors[processor_id]
    poison_injector = proc_data.get('poison_injector')
    
    if not poison_injector:
        return jsonify({
            'enabled': False,
            'message': 'Poison injection not enabled for this processor'
        })
    
    stats = poison_injector.get_injection_stats()
    return jsonify({
        'enabled': True,
        'stats': stats,
        'injected_poisons': proc_data.get('injected_poisons', [])
    })


@app.route('/api/process/<processor_id>/export', methods=['GET'])
def export_anomalies_csv(processor_id):
    """Export all detected anomalies for a processor as CSV."""
    if processor_id not in active_processors:
        return jsonify({'error': 'Processor not found'}), 404

    proc_data = active_processors[processor_id]
    anomalies = proc_data.get('all_anomalies', [])

    # Build CSV in memory
    from io import StringIO
    import csv

    output = StringIO()
    writer = csv.writer(output)
    writer.writerow(['timestamp', 'severity', 'reasons', 'lat', 'lon'])

    for a in anomalies:
        pos = a.get('position', {}) or {}
        writer.writerow([
            a.get('timestamp', ''),
            a.get('severity', ''),
            ' | '.join(a.get('reasons', [])),
            pos.get('lat', ''),
            pos.get('lon', ''),
        ])

    csv_data = output.getvalue()
    return csv_data, 200, {
        'Content-Type': 'text/csv',
        'Content-Disposition': 'attachment; filename=\"anomalies.csv\"'
    }


@app.route('/api/upload', methods=['POST'])
def upload_file():
    """Handle ROS bag file upload (supports .bag and .zip)"""
    if 'file' not in request.files:
        return jsonify({'error': 'No file provided'}), 400
    
    file = request.files['file']
    if file.filename == '':
        return jsonify({'error': 'No file selected'}), 400
    
    # Save uploaded file
    upload_dir = Path('uploads')
    upload_dir.mkdir(exist_ok=True)
    
    file_path = upload_dir / file.filename
    file.save(str(file_path))
    
    print(f"[UPLOAD] Uploaded file: {file.filename}")
    
    # Check if it's a zip file (ROS2 bag folder)
    if file.filename.endswith('.zip'):
        print(f"[UPLOAD] Detected zip file, extracting...")
        extract_dir = upload_dir / file.filename.replace('.zip', '')
        extract_dir.mkdir(exist_ok=True)
        
        try:
            with zipfile.ZipFile(file_path, 'r') as zip_ref:
                zip_ref.extractall(extract_dir)
            
            # Look for ROS2 bag folder
            bag_folder = None
            for item in extract_dir.rglob('metadata.yaml'):
                bag_folder = item.parent
                print(f"[UPLOAD] Found ROS2 bag folder: {bag_folder}")
                break
            
            if bag_folder:
                return jsonify({
                    'filePath': str(bag_folder),
                    'filename': file.filename,
                    'bagType': 'ROS2',
                    'message': 'ROS2 bag folder extracted successfully'
                })
            else:
                return jsonify({'error': 'No valid ROS2 bag found in zip'}), 400
        
        except Exception as e:
            print(f"[UPLOAD ERROR] Failed to extract: {e}")
            return jsonify({'error': f'Failed to extract: {str(e)}'}), 400
    
    # Regular .bag file
    elif file.filename.endswith('.bag'):
        return jsonify({
            'filePath': str(file_path),
            'filename': file.filename,
            'bagType': 'ROS1',
            'message': 'ROS1 bag file uploaded successfully'
        })
    
    else:
        return jsonify({'error': 'Unsupported file format'}), 400


if __name__ == '__main__':
    # Create uploads directory
    Path('uploads').mkdir(exist_ok=True)
    
    print("=" * 60)
    print("  ROS Sensor Data Poisoning Detection Dashboard")
    print("=" * 60)
    print("\n  Open: http://localhost:5000")
    print("\n  Features:")
    print("    • Poison injection controls")
    print("    • Anomaly detection validation")
    print("    • ROS1 (.bag) and ROS2 (.zip) support")
    print("    • Real-time monitoring")
    print("\n  Press Ctrl+C to stop")
    print("=" * 60)
    
    app.run(debug=True, host='0.0.0.0', port=5000)

