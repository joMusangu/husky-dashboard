"""
Flask API Server for ROS Sensor Data Poisoning Detection
Uses HTTP polling for real-time updates (no WebSocket needed)
"""

from flask import Flask, request, jsonify, send_from_directory
from flask_cors import CORS
import os
import threading
from pathlib import Path
import json
import zipfile
import time

# Import backend classes
from backend import (
    ROSBagParser, AnomalyDetector, 
    RealtimeProcessor, SensorReading
)
from poison_injector import (
    PoisonInjector, PoisonValidator, PoisonConfig, PoisonType,
    get_preset_attack, list_preset_attacks
)
import numpy as np

app = Flask(__name__, static_folder='.')

# Configure CORS
CORS(app, resources={r"/*": {"origins": "*"}})

# Global poison injection and validation instances
poison_injector = PoisonInjector()
poison_validator = PoisonValidator()

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
    return send_from_directory('.', 'advanced_dashboard.html')


@app.route('/api/health', methods=['GET'])
def health():
    """Health check endpoint"""
    return jsonify({
        'status': 'healthy',
        'message': 'ROS Sensor Data Poisoning Detection API is running'
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
        if poison_injector.active_poisons:
            reading, is_poisoned, poison_info = poison_injector.inject(reading, t)
            if is_poisoned:
                print(f"[POISON] Injected at t={t:.2f}s: {poison_info['applied_poisons']}")
        else:
            # Default poisoning for demonstration
            if 80 <= i < 120:
                reading.gps_lat += 0.0005
                reading.gps_lon += 0.0005
        
        readings.append(reading)
        processor.add_reading(reading)
        proc_data['readings'].append(reading)
        
        # Get results
        results = processor.get_results()
        proc_data['results'].extend(results)
        
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
            if poison_injector.active_poisons:
                reading, is_poisoned, poison_info = poison_injector.inject(reading, reading.timestamp)
                if is_poisoned and i % 10 == 0:  # Log every 10th injection to avoid spam
                    print(f"[POISON] Injected at t={reading.timestamp:.2f}s: {poison_info['applied_poisons']}")
            
            proc_data['readings'].append(reading)
            processor.add_reading(reading)
            
            # Get results
            results = processor.get_results()
            proc_data['results'].extend(results)
            
            # Log progress every 100 readings
            if (i + 1) % 100 == 0:
                print(f"[PROCESSOR {processor_id}] Processed {i + 1}/{len(readings)} readings")
            
            time.sleep(0.05)  # 50ms per reading
        
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
    
    with processor_lock:
        active_processors[processor_id] = {
            'processor': processor,
            'detector': detector,
            'readings': [],
            'results': [],
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
    
    # Get latest results
    latest_results = processor.get_results()
    results.extend(latest_results)
    
    # Calculate metrics
    anomalies = [r for r in results if r.get('anomaly') is not None]
    total_readings = len(readings)
    
    # Get trust scores
    trust_scores = detector.trust_manager.get_scores()
    
    # Get latest reading
    latest_reading = readings[-1] if readings else None
    
    # Format sensor data
    sensor_data = {}
    for sensor, score in trust_scores.items():
        status = 'healthy' if score > 0.8 else ('warning' if score > 0.5 else 'critical')
        sensor_data[sensor] = {
            'status': status,
            'trustScore': score,
            'lastReading': latest_reading.timestamp if latest_reading else None
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
    
    # Format anomalies
    formatted_anomalies = []
    for result in anomalies:
        anomaly = result.get('anomaly')
        if anomaly:
            formatted_anomalies.append({
                'timestamp': anomaly['timestamp'],
                'severity': anomaly['severity'],
                'reasons': anomaly['reasons'],
                'position': anomaly['position']
            })
    
    # Calculate processing time
    avg_processing_time = 0
    if results:
        processing_times = [r.get('processing_time', 0) for r in results]
        avg_processing_time = sum(processing_times) / len(processing_times) * 1000
    
    response = {
        'isProcessing': proc_data['running'],
        'metrics': {
            'totalReadings': total_readings,
            'anomaliesDetected': len(anomalies),
            'detectionRate': (len(anomalies) / total_readings * 100) if total_readings > 0 else 0,
            'avgProcessingTime': avg_processing_time,
            'currentTimestamp': latest_reading.timestamp if latest_reading else 0
        },
        'sensorData': sensor_data,
        'trajectoryData': trajectory_data,
        'anomalies': formatted_anomalies
    }
    
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


# ============================================================================
# POISON INJECTION & VALIDATION ENDPOINTS
# ============================================================================

@app.route('/api/poison/presets', methods=['GET'])
def get_preset_attacks():
    """Get list of preset attack scenarios"""
    presets = list_preset_attacks()
    descriptions = {
        'quick_test': 'Two GPS jumps at 2s and 5s (50m & 75m)',
        'sustained_drift': 'Gradual 5m/s GPS drift for 5 seconds',
        'sensor_freeze': 'GPS readings freeze for 3 seconds',
        'imu_attack': 'IMU noise + bias attacks',
        'multi_sensor': 'Combined GPS, IMU, and Odometry attacks',
        'intermittent': 'Four rapid GPS jumps at intervals'
    }
    
    return jsonify({
        'presets': presets,
        'descriptions': descriptions
    })


@app.route('/api/poison/inject', methods=['POST'])
def inject_poison():
    """Inject a poisoning attack"""
    data = request.json
    
    # Clear existing poisons if requested
    if data.get('clear_existing', False):
        poison_injector.clear_poisons()
    
    # Check if using preset
    if 'preset' in data:
        preset_name = data['preset']
        preset_poisons = get_preset_attack(preset_name)
        
        if not preset_poisons:
            return jsonify({'error': f'Unknown preset: {preset_name}'}), 400
        
        for poison in preset_poisons:
            poison_injector.add_poison(poison)
        
        return jsonify({
            'status': 'success',
            'message': f'Added {len(preset_poisons)} poisons from preset "{preset_name}"',
            'poisons_added': len(preset_poisons),
            'active_poisons': len(poison_injector.active_poisons)
        })
    
    # Custom poison configuration
    try:
        poison_type = PoisonType(data['poison_type'])
        config = PoisonConfig(
            poison_type=poison_type,
            start_time=data.get('start_time', 0.0),
            duration=data.get('duration', 1.0),
            intensity=data.get('intensity', 1.0),
            target_sensor=data.get('target_sensor', 'gps'),
            jump_distance=data.get('jump_distance', 50.0),
            drift_rate=data.get('drift_rate', 5.0),
            noise_stddev=data.get('noise_stddev', 0.1),
            bias_value=data.get('bias_value', 0.5),
            scale_factor=data.get('scale_factor', 2.0)
        )
        
        poison_injector.add_poison(config)
        
        return jsonify({
            'status': 'success',
            'message': f'Added {poison_type.value} poison',
            'active_poisons': len(poison_injector.active_poisons)
        })
        
    except (KeyError, ValueError) as e:
        return jsonify({'error': f'Invalid poison configuration: {str(e)}'}), 400


@app.route('/api/poison/clear', methods=['POST'])
def clear_poisons():
    """Clear all active poisoning configurations"""
    poison_injector.clear_poisons()
    
    return jsonify({
        'status': 'success',
        'message': 'All poisons cleared'
    })


@app.route('/api/poison/status', methods=['GET'])
def poison_status():
    """Get current poison injection status"""
    stats = poison_injector.get_injection_stats()
    
    return jsonify({
        'status': 'active' if stats['active_poisons'] > 0 else 'inactive',
        'statistics': stats
    })


@app.route('/api/poison/validate', methods=['POST'])
def validate_poisons():
    """Validate that injected poisons were detected"""
    data = request.json
    processor_id = data.get('processor_id')
    
    if not processor_id or processor_id not in active_processors:
        return jsonify({'error': 'Invalid or missing processor_id'}), 400
    
    proc_data = active_processors[processor_id]
    results = proc_data['results']
    
    # Get injection log
    injection_log = poison_injector.injection_log
    
    # Get detected anomalies
    detected_anomalies = [r for r in results if r.get('anomaly') is not None]
    
    # Validate
    validation_result = poison_validator.validate_detection(injection_log, detected_anomalies)
    
    return jsonify({
        'status': 'success',
        'validation': validation_result,
        'summary': poison_validator.get_validation_summary()
    })


@app.route('/api/poison/types', methods=['GET'])
def get_poison_types():
    """Get available poison types and their descriptions"""
    types = {
        'gps_jump': 'Sudden GPS position jump (configurable distance)',
        'gps_drift': 'Gradual GPS position drift (configurable rate)',
        'gps_freeze': 'GPS readings freeze at current position',
        'imu_noise': 'Add random noise to IMU readings',
        'imu_bias': 'Add constant bias to IMU readings',
        'odom_scaling': 'Scale odometry velocity values',
        'velocity_spike': 'Sudden velocity spike in odometry',
        'altitude_jump': 'Sudden altitude jump in GPS',
        'sensor_dropout': 'Sensor stops reporting (not yet implemented)',
        'replay_attack': 'Replay old sensor data (not yet implemented)'
    }
    
    return jsonify({
        'poison_types': types,
        'available': list(types.keys())
    })


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
