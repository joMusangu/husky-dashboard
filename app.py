"""
Flask API Server for ROS Sensor Data Poisoning Detection
Uses HTTP polling for real-time updates (no WebSocket needed)
"""

from flask import Flask, request, jsonify, send_from_directory, Response
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
import pandas as pd
from collections import deque

# GPS Prediction Model
try:
    from gps_prediction_model import GPSPredictionModel, haversine_distance
    GPS_PREDICTION_AVAILABLE = True
except ImportError as e:
    GPS_PREDICTION_AVAILABLE = False
    haversine_distance = None
    print(f"⚠ Warning: GPS prediction model not available: {e}")

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

# GPS Prediction Model (loaded once)
gps_predictor = None
if GPS_PREDICTION_AVAILABLE:
    try:
        gps_predictor = GPSPredictionModel(
            prediction_interval=5.0,
            model_type='lstm',
            model_dir='gps_models'
        )
        gps_predictor.load_model()
        print("✓ GPS prediction model loaded successfully")
        print(f"  Model type: {gps_predictor.model_type}")
        print(f"  Prediction interval: {gps_predictor.prediction_interval}s")
        print(f"  Model directory: {gps_predictor.model_dir}")
    except Exception as e:
        print(f"⚠ Warning: Could not load GPS prediction model: {e}")
        import traceback
        traceback.print_exc()
        gps_predictor = None


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
        # processor.add_reading(reading)  # Disabled - anomaly detection not used
        proc_data['readings'].append(reading)
        
        # Get results (anomaly detection disabled)
        # results = processor.get_results()
        # proc_data['results'].extend(results)
        results = []  # Empty results - anomaly detection disabled
        
        time.sleep(0.005)  # 5ms per reading for faster visualization (10x speed)
    
    # Wait for processing to complete
    time.sleep(0.5)
    # processor.stop()  # Disabled - processor not used
    proc_data['running'] = False
    
    print(f"[PROCESSOR {processor_id}] Synthetic data processing complete: {len(readings)} readings")


def process_ros_bag_resume(processor_id, bag_path, config, start_index):
    """Resume processing ROS bag file from a specific index"""
    print(f"[PROCESSOR {processor_id}] Resuming bag file processing from index {start_index}")
    
    if processor_id not in active_processors:
        return
    
    proc_data = active_processors[processor_id]
    
    # Get cached readings or parse again
    if 'cached_readings' in proc_data:
        readings = proc_data['cached_readings']
    else:
        parser = ROSBagParser()
        readings = parser.parse_bag(str(bag_path))
        proc_data['cached_readings'] = readings
    
    # Continue processing from start_index using the same function
    # We'll modify process_ros_bag to handle resume properly
    process_ros_bag(processor_id, bag_path, config, start_index=start_index)


def process_ros_bag(processor_id, bag_path, config, start_index=0):
    """Process actual ROS bag file"""
    print(f"[PROCESSOR {processor_id}] Processing bag file: {bag_path}")
    
    if processor_id not in active_processors:
        return
    
    proc_data = active_processors[processor_id]
    processor = proc_data['processor']
    parser = ROSBagParser()
    
    # Store bag path immediately for resume functionality
    proc_data['bag_path'] = bag_path
    proc_data['config'] = config
    
    try:
        # Verify path exists
        if not Path(bag_path).exists():
            error_msg = f"Bag file not found: {bag_path}"
            print(f"[ERROR] {error_msg}")
            proc_data['error'] = error_msg
            # processor.stop()  # Disabled - processor not used
            proc_data['running'] = False
            return
        
        # Parse the bag file (or use cached if resuming)
        if start_index > 0 and 'cached_readings' in proc_data:
            print(f"[PROCESSOR {processor_id}] Using cached readings for resume from index {start_index}")
            readings = proc_data['cached_readings']
        else:
            print(f"[PROCESSOR {processor_id}] Parsing bag file...")
            readings = parser.parse_bag(str(bag_path))
        
        if not readings:
            error_msg = "No sensor readings extracted. Check topic names."
            print(f"[WARNING] {error_msg}")
            proc_data['error'] = error_msg
                # processor.stop()  # Disabled - processor not used
            proc_data['running'] = False
            return
        
            # Store bag path and cache readings for resume functionality
            proc_data['bag_path'] = bag_path
            proc_data['cached_readings'] = readings  # Cache for resume
        
        print(f"[PROCESSOR {processor_id}] Starting processing from index {start_index} (total readings: {len(readings)})...")
        
        # Initialize simple GPS prediction data structures
        predictions = {}  # Dictionary: {prediction_time: {predicted_lat, predicted_lon, timestamp, ...}}
        detected_anomalies = proc_data.get('anomalies', [])  # Keep existing anomalies
        # Get anomaly threshold from proc_data (can be updated via API)
        anomaly_threshold_degrees = proc_data.get('anomaly_threshold_degrees', config.get('anomaly_threshold_degrees', 2.0))
        proc_data['anomaly_threshold_degrees'] = anomaly_threshold_degrees
        prediction_interval = 1.0  # Predict 1 second ahead
        should_stop = proc_data.get('should_stop', False)  # Flag to stop processing when anomaly detected
        stop_index = None  # Store index where processing stopped
        
        # Store GPS and IMU data for prediction
        last_gps_time = None  # Track last GPS prediction time
        last_gps_coord = None  # Store last GPS coordinate used for prediction
        
        # Store last 5 seconds of motion data for anomaly display
        motion_history = []  # Store GPS, IMU, and prediction data for last 5 seconds
        anomaly_motion_data = proc_data.get('anomaly_motion_data', None)  # Keep existing if resuming
        
        # Process each reading (starting from start_index if resuming)
        for i, reading in enumerate(readings[start_index:], start=start_index):
            if processor_id not in active_processors or not proc_data['running'] or should_stop:
                if should_stop:
                    print(f"[STOP] Processing stopped at reading {i+1}/{len(readings)} due to critical GPS anomaly")
                    # Store where we stopped for resume functionality
                    proc_data['resume_index'] = i
                break
            
            # Apply poison injection if configured
            if poison_injector.active_poisons:
                reading, is_poisoned, poison_info = poison_injector.inject(reading, reading.timestamp)
                if is_poisoned and i % 10 == 0:  # Log every 10th injection to avoid spam
                    print(f"[POISON] Injected at t={reading.timestamp:.2f}s: {poison_info['applied_poisons']}")
            
            proc_data['readings'].append(reading)
            # processor.add_reading(reading)  # Disabled - anomaly detection not used
            
            # Simple GPS prediction: At s=0 take GPS, use IMU to predict at s=1, compare
            if reading.gps_lat and reading.gps_lon:
                current_time = reading.timestamp
                current_lat = reading.gps_lat
                current_lon = reading.gps_lon
                
                # Store motion data for last 5 seconds (only if we have valid GPS)
                motion_data_point = {
                    'timestamp': current_time,
                    'gps_lat': current_lat,
                    'gps_lon': current_lon,
                    'velocity': reading.velocity if reading.velocity else 0.0,
                    'imu_angular_velocity': reading.imu_angular_velocity if reading.imu_angular_velocity else None,
                    'imu_linear_acceleration': reading.imu_linear_acceleration if reading.imu_linear_acceleration else None,
                    'prediction': None  # Will be filled when prediction is made
                }
                motion_history.append(motion_data_point)
                
                # Keep only last 5 seconds of data
                cutoff_time = current_time - 5.0
                motion_history = [m for m in motion_history if m['timestamp'] >= cutoff_time]
                
                # Check if it's time to make a new prediction (every 1 second)
                should_predict = False
                if last_gps_time is None:
                    # First GPS reading - use it as starting point
                    last_gps_time = current_time
                    last_gps_coord = (current_lat, current_lon)
                    should_predict = True
                    print(f"[PRED_START] At t={current_time:.2f}s: Starting GPS coordinate: ({current_lat:.6f}, {current_lon:.6f})")
                elif (current_time - last_gps_time) >= prediction_interval:
                    # It's been 1 second since last prediction - make a new one
                    should_predict = True
                
                # Make prediction at 1-second intervals
                if should_predict and last_gps_coord:
                    # Get IMU data for velocity and direction
                    # Use velocity from reading if available
                    velocity_mps = 0.0
                    if reading.velocity is not None:
                        velocity_mps = reading.velocity
                    
                    # Get direction from IMU or calculate from GPS movement
                    direction_rad = 0.0
                    
                    # Try to get direction from IMU orientation or angular velocity
                    if reading.imu_angular_velocity and len(reading.imu_angular_velocity) >= 3:
                        # Use yaw rate (z-axis angular velocity) for direction change
                        angular_z = reading.imu_angular_velocity[2]
                        # For 1 second prediction, use current angular velocity
                        direction_rad = angular_z * prediction_interval
                    
                    # If no angular velocity, try to calculate direction from GPS movement
                    # (This would require previous GPS, but we're using last_gps_coord which is from last prediction)
                    # For now, if velocity is available, assume forward direction (0 radians = North)
                    # In practice, you'd track heading from IMU orientation quaternion
                    
                    # Convert velocity (m/s) to degrees per second
                    # 1 degree latitude ≈ 111,320 meters
                    # 1 degree longitude ≈ 111,320 * cos(latitude) meters
                    meters_per_degree_lat = 111320.0
                    meters_per_degree_lon = 111320.0 * np.cos(np.radians(last_gps_coord[0]))
                    
                    # Calculate velocity components in degrees per second
                    # Use velocity and direction to calculate lat/lon velocity
                    if abs(velocity_mps) > 0.001:  # If we have significant velocity
                        # Calculate velocity in degrees per second
                        lat_velocity_deg_per_s = (velocity_mps * np.cos(direction_rad)) / meters_per_degree_lat
                        lon_velocity_deg_per_s = (velocity_mps * np.sin(direction_rad)) / meters_per_degree_lon
                    else:
                        # No velocity - assume stationary
                        lat_velocity_deg_per_s = 0.0
                        lon_velocity_deg_per_s = 0.0
                    
                    # Predict GPS position 1 second ahead
                    future_time = last_gps_time + prediction_interval
                    predicted_lat = last_gps_coord[0] + (lat_velocity_deg_per_s * prediction_interval)
                    predicted_lon = last_gps_coord[1] + (lon_velocity_deg_per_s * prediction_interval)
                    
                    # Store prediction
                    pred_data = {
                        'timestamp': last_gps_time,
                        'prediction_time': future_time,
                        'current_lat': last_gps_coord[0],
                        'current_lon': last_gps_coord[1],
                        'predicted_lat': predicted_lat,
                        'predicted_lon': predicted_lon,
                        'velocity_mps': velocity_mps,
                        'direction_rad': direction_rad
                    }
                    predictions[future_time] = pred_data
                    
                    # Store prediction in motion history
                    for m in motion_history:
                        if abs(m['timestamp'] - last_gps_time) < 0.1:
                            m['prediction'] = pred_data
                            break
                    
                    # Update for next prediction
                    last_gps_time = current_time
                    last_gps_coord = (current_lat, current_lon)
                    
                    # Log prediction
                    print(f"[PRED] At t={last_gps_time:.2f}s: Predicted GPS for t={future_time:.2f}s")
                    print(f"  Start GPS: ({last_gps_coord[0]:.6f}, {last_gps_coord[1]:.6f})")
                    print(f"  Predicted: ({predicted_lat:.6f}, {predicted_lon:.6f})")
                    print(f"  Velocity: {velocity_mps:.2f} m/s, Direction: {np.degrees(direction_rad):.2f}°")
                
                # Check if current GPS reading matches a previous prediction (at s=1) - for anomaly detection
                prediction_window = 0.3  # Allow 0.3s tolerance for 1-second predictions
                matching_prediction = None
                
                for pred_time, pred_data in predictions.items():
                    time_diff = abs(current_time - pred_time)
                    if time_diff <= prediction_window:
                        matching_prediction = pred_data
                        break
                
                # ALWAYS get the most recent prediction for trust score calculation (regardless of timing)
                # This ensures trust score updates on EVERY GPS reading if any prediction exists
                most_recent_prediction = None
                if predictions:
                    # Get the most recent prediction (closest to current time, no time limit)
                    closest_time = min(predictions.keys(), key=lambda t: abs(t - current_time))
                    most_recent_prediction = predictions[closest_time]
                
                # Use matching prediction for anomaly check, but use most recent for trust score
                # This ensures trust score updates continuously, not just when predictions match exactly
                prediction_to_compare = most_recent_prediction  # Always use most recent for trust score
                
                # Calculate trust score for EVERY GPS reading if we have any prediction
                # This happens regardless of whether it's an anomaly or not
                if prediction_to_compare:
                    # Calculate difference in degrees (latitude and longitude separately)
                    lat_diff_degrees = abs(prediction_to_compare['predicted_lat'] - current_lat)
                    lon_diff_degrees = abs(prediction_to_compare['predicted_lon'] - current_lon)
                    
                    # Calculate trust score using percentage difference method
                    # Calculate % difference between actual and predicted for both lat and lon, then average
                    predicted_lat = prediction_to_compare['predicted_lat']
                    predicted_lon = prediction_to_compare['predicted_lon']
                    
                    # Initialize variables
                    lat_error_pct = 0.0
                    lon_error_pct = 0.0
                    average_error_pct = 0.0
                    
                    # Calculate percentage difference: |actual - predicted| / |predicted| * 100
                    # Avoid division by zero
                    if abs(predicted_lat) > 0.000001:
                        lat_error_pct = abs((current_lat - predicted_lat) / predicted_lat) * 100
                    else:
                        lat_error_pct = 100.0  # If predicted is zero, 100% error
                    
                    if abs(predicted_lon) > 0.000001:
                        lon_error_pct = abs((current_lon - predicted_lon) / predicted_lon) * 100
                    else:
                        lon_error_pct = 100.0  # If predicted is zero, 100% error
                    
                    # Average the error percentages
                    average_error_pct = (lat_error_pct + lon_error_pct) / 2.0
                    
                    # Trust score: 100% - average_error_percentage (clamped between 0 and 100)
                    # Reset each time - always based on current error, not accumulated
                    trust_score = max(0, min(100, 100 - average_error_pct))
                    error_percentage = average_error_pct
                    
                    # Store latest trust score in proc_data (resets each time) - ALWAYS UPDATE
                    proc_data['latest_trust_score'] = trust_score
                    proc_data['latest_error_percentage'] = error_percentage
                    proc_data['latest_prediction_comparison'] = {
                        'timestamp': current_time,
                        'lat_diff_degrees': lat_diff_degrees,
                        'lon_diff_degrees': lon_diff_degrees,
                        'lat_error_pct': lat_error_pct,
                        'lon_error_pct': lon_error_pct,
                        'average_error_pct': average_error_pct,
                        'error_percentage': error_percentage,
                        'trust_score': trust_score
                    }
                    
                    # Log trust score update (every time, not just anomalies)
                    print(f"[TRUST_UPDATE] At t={current_time:.2f}s: Trust Score = {trust_score:.2f}%")
                    print(f"  Predicted: ({predicted_lat:.6f}, {predicted_lon:.6f})")
                    print(f"  Actual:    ({current_lat:.6f}, {current_lon:.6f})")
                    print(f"  Error: {average_error_pct:.3f}% (Lat: {lat_error_pct:.3f}%, Lon: {lon_error_pct:.3f}%)")
                
                # Only check for anomalies if we have a matching prediction (exact time match)
                if matching_prediction:
                    # Recalculate differences for anomaly check (in case we used most_recent_prediction above)
                    lat_diff_degrees = abs(matching_prediction['predicted_lat'] - current_lat)
                    lon_diff_degrees = abs(matching_prediction['predicted_lon'] - current_lon)
                    
                    # Check if either latitude or longitude is 2+ degrees away
                    lat_exceeds = lat_diff_degrees >= anomaly_threshold_degrees
                    lon_exceeds = lon_diff_degrees >= anomaly_threshold_degrees
                    exceeds_threshold = lat_exceeds or lon_exceeds
                    
                    # Log comparison
                    status = "✓ WITHIN" if not exceeds_threshold else "⚠ OUTSIDE"
                    print(f"[COMPARE] At t={current_time:.2f}s: {status} boundary")
                    print(f"  Lat diff: {lat_diff_degrees:.6f}° (threshold: {anomaly_threshold_degrees}°)")
                    print(f"  Lon diff: {lon_diff_degrees:.6f}° (threshold: {anomaly_threshold_degrees}°)")
                    
                    # Flag as anomaly and STOP if outside boundary (2+ degrees)
                    if exceeds_threshold:
                        # Check if we already detected this anomaly (avoid duplicates)
                        is_duplicate = any(
                            abs(a['timestamp'] - current_time) < 0.3 
                            for a in detected_anomalies
                        )
                        
                        if not is_duplicate:
                            # Save last 5 seconds of motion data
                            anomaly_motion_data = {
                                'motion_history': motion_history.copy(),
                                'anomaly_timestamp': current_time,
                                'anomaly_lat': current_lat,
                                'anomaly_lon': current_lon,
                                'predicted_lat': matching_prediction['predicted_lat'],
                                'predicted_lon': matching_prediction['predicted_lon'],
                                'lat_diff_degrees': lat_diff_degrees,
                                'lon_diff_degrees': lon_diff_degrees
                            }
                            
                            anomaly = {
                                'timestamp': current_time,
                                'severity': 'critical',
                                'reasons': [
                                    f'GPS jump detected: {lat_diff_degrees:.6f}° latitude difference (threshold: {anomaly_threshold_degrees}°)',
                                    f'GPS jump detected: {lon_diff_degrees:.6f}° longitude difference (threshold: {anomaly_threshold_degrees}°)'
                                ],
                                'position': {
                                    'lat': current_lat,
                                    'lon': current_lon
                                },
                                'predicted_position': {
                                    'lat': matching_prediction['predicted_lat'],
                                    'lon': matching_prediction['predicted_lon']
                                },
                                'lat_diff_degrees': lat_diff_degrees,
                                'lon_diff_degrees': lon_diff_degrees,
                                'type': 'gps_jump_critical',
                                'motion_data': anomaly_motion_data
                            }
                            detected_anomalies.append(anomaly)
                            print(f"[ANOMALY] 🚨 CRITICAL GPS ANOMALY DETECTED at t={current_time:.2f}s")
                            print(f"  Actual GPS:    ({current_lat:.6f}, {current_lon:.6f})")
                            print(f"  Predicted GPS: ({matching_prediction['predicted_lat']:.6f}, {matching_prediction['predicted_lon']:.6f})")
                            print(f"  Lat difference: {lat_diff_degrees:.6f}° (threshold: {anomaly_threshold_degrees}°)")
                            print(f"  Lon difference: {lon_diff_degrees:.6f}° (threshold: {anomaly_threshold_degrees}°)")
                            print(f"[STOP] Stopping processing due to critical GPS anomaly!")
                            print(f"[STOP] Saved last 5 seconds of motion data ({len(motion_history)} data points)")
                            
                            # Store stop index and motion data
                            stop_index = i
                            proc_data['stop_index'] = stop_index
                            proc_data['anomaly_motion_data'] = anomaly_motion_data
                            
                            # Set flag to stop processing
                            should_stop = True
                            proc_data['running'] = False
            
            # Get results (anomaly detection disabled)
            results = []  # Empty results - anomaly detection disabled
            
            # Store predictions (convert dict to list for frontend) and anomalies
            predictions_list = list(predictions.values())
            # Keep only recent predictions for display (last 10)
            proc_data['predictions'] = predictions_list[-10:] if len(predictions_list) > 10 else predictions_list
            proc_data['anomalies'] = detected_anomalies
            
            # Log progress every 100 readings
            if (i + 1) % 100 == 0:
                print(f"[PROCESSOR {processor_id}] Processed {i + 1}/{len(readings)} readings")
            
            time.sleep(0.005)  # 5ms per reading for faster playback (10x speed)
        
        time.sleep(0.5)
        # processor.stop()  # Disabled - processor not used
        proc_data['running'] = False
        
        print(f"[PROCESSOR {processor_id}] Bag processing complete: {len(proc_data['readings'])} readings")
        
    except Exception as e:
        print(f"[ERROR] Error processing bag: {e}")
        import traceback
        traceback.print_exc()
        proc_data['error'] = str(e)
        # processor.stop()  # Disabled - processor not used
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
    
    # Create detector and processor (anomaly detection disabled)
    # prediction_window = config.get('predictionWindow', 5)
    # detector = AnomalyDetector(prediction_window=prediction_window)
    # processor = RealtimeProcessor(detector)
    detector = None  # Anomaly detection disabled
    processor = None  # Processor disabled
    
    with processor_lock:
        active_processors[processor_id] = {
            'processor': processor,
            'detector': detector,
            'readings': [],
            'results': [],
            'predictions': [],
            'anomalies': [],
            'config': config,
            'bag_path': file_path if file_path else None,  # Store bag path immediately
            'anomaly_threshold_degrees': config.get('anomaly_threshold_degrees', 2.0),  # Default threshold
            'running': True,
            'error': None
        }
    
    # processor.start()  # Disabled - anomaly detection not used
    
    # Start processing in background thread
    if use_synthetic or not file_path:
        thread = threading.Thread(
            target=process_synthetic_data,
            args=(processor_id, config)
        )
    else:
        thread = threading.Thread(
            target=process_ros_bag,
            args=(processor_id, file_path, config, 0)  # Start from index 0
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
    
    # Get latest results (anomaly detection disabled)
    # latest_results = processor.get_results() if processor else []
    # results.extend(latest_results)
    latest_results = []  # Anomaly detection disabled
    
    # Format anomalies from GPS prediction errors (must be defined before use)
    formatted_anomalies = []
    if 'anomalies' in proc_data and proc_data['anomalies']:
        for anomaly in proc_data['anomalies']:
            formatted_anomalies.append({
                'timestamp': anomaly['timestamp'],
                'severity': anomaly.get('severity', 'medium'),
                'reasons': anomaly.get('reasons', []),
                'position': anomaly.get('position', {}),
                'error_meters': anomaly.get('error_meters', 0),
                'type': anomaly.get('type', 'gps_jump')
            })
    
    # Calculate metrics from GPS prediction anomalies
    anomalies = formatted_anomalies  # Use GPS prediction-based anomalies
    total_readings = len(readings)
    
    # Get trust scores (anomaly detection disabled)
    # trust_scores = detector.trust_manager.get_scores() if detector else {}
    trust_scores = {}  # Anomaly detection disabled
    
    # Get latest reading
    latest_reading = readings[-1] if readings else None
    
    # Calculate trust scores based on latest prediction error (resets each time)
    sensor_data = {}
    
    # Always calculate trust score from latest_trust_score if available (updated on every GPS reading)
    # This ensures trust score updates dynamically with every prediction comparison
    if 'latest_trust_score' in proc_data and proc_data['latest_trust_score'] is not None:
        trust_score = proc_data['latest_trust_score']
        error_percentage = proc_data.get('latest_error_percentage', 0.0)
        
        print(f"[TRUST_SCORE] Latest trust score: {trust_score:.2f}% (error: {error_percentage:.2f}%)")
        
        # GPS trust score based on prediction accuracy
        sensor_data['gps'] = {
            'trustScore': trust_score / 100.0,  # Convert to 0-1 range
            'errorPercentage': error_percentage
        }
    elif 'latest_prediction_comparison' in proc_data:
        # Fallback to latest_prediction_comparison if latest_trust_score not available
        comparison = proc_data['latest_prediction_comparison']
        trust_score = comparison.get('trust_score', 100.0)
        error_percentage = comparison.get('error_percentage', 0.0)
        
        print(f"[TRUST_SCORE] Using comparison trust score: {trust_score:.2f}% (error: {error_percentage:.2f}%)")
        
        # GPS trust score based on prediction accuracy
        sensor_data['gps'] = {
            'trustScore': trust_score / 100.0,  # Convert to 0-1 range
            'errorPercentage': error_percentage
        }
    else:
        # No prediction comparison yet - default to 100% trust
        print(f"[TRUST_SCORE] No prediction comparison available, using default 100%")
        sensor_data['gps'] = {
            'trustScore': 1.0,
            'errorPercentage': 0.0
        }
    
    # Format trajectory data with anomaly flags
    trajectory_data = []
    anomaly_timestamps = {a['timestamp'] for a in formatted_anomalies}
    for reading in readings:
        # Check if this reading has an anomaly
        is_anomaly = any(abs(a_ts - reading.timestamp) < 0.1 for a_ts in anomaly_timestamps)
        trajectory_data.append({
            'lat': reading.gps_lat,
            'lon': reading.gps_lon,
            'timestamp': reading.timestamp,
            'isPoisoned': is_anomaly  # Flag GPS jumps as poisoned
        })
    
    # Calculate processing time (anomaly detection disabled)
    avg_processing_time = 0
    # Processing time not available - anomaly detection disabled
    
    # Get GPS predictions - always return the latest prediction
    predictions_data = []
    if 'predictions' in proc_data and proc_data['predictions'] and len(proc_data['predictions']) > 0:
        # Get the latest prediction
        latest_pred = proc_data['predictions'][-1]
        if latest_pred and 'predicted_lat' in latest_pred and 'predicted_lon' in latest_pred:
            predictions_data = [{
                'timestamp': latest_pred.get('timestamp', 0),
                'current_lat': latest_pred.get('current_lat', 0),
                'current_lon': latest_pred.get('current_lon', 0),
                'predicted_lat': latest_pred['predicted_lat'],
                'predicted_lon': latest_pred['predicted_lon'],
                'error_meters': latest_pred.get('error_meters', 0),
                'prediction_time': latest_pred.get('prediction_time', 0)
            }]
            # Debug: Log prediction availability
            print(f"[STATUS] Returning prediction: t={latest_pred.get('timestamp', 0):.2f}s, pred=({latest_pred['predicted_lat']:.6f}, {latest_pred['predicted_lon']:.6f})")
    else:
        print(f"[STATUS] No predictions available. Total predictions: {len(proc_data.get('predictions', []))}")
    
    response = {
        'isProcessing': proc_data['running'],
        'metrics': {
            'totalReadings': total_readings,
            'anomaliesDetected': len(anomalies),
            'detectionRate': (len(anomalies) / total_readings * 100) if total_readings > 0 else 0,
            'avgProcessingTime': avg_processing_time,
            'currentTimestamp': latest_reading.timestamp if latest_reading else 0,
            'gpsAnomalyThreshold': proc_data.get('anomaly_threshold_degrees', 2.0)  # Threshold in degrees
        },
        'sensorData': sensor_data,
        'trajectoryData': trajectory_data,
        'anomalies': formatted_anomalies,
        'predictions': predictions_data,
        'hasAnomalyData': 'anomaly_motion_data' in proc_data,
        'isStopped': not proc_data['running'] and 'stop_index' in proc_data
    }
    
    if proc_data.get('error'):
        response['error'] = proc_data['error']
    
    # Include anomaly motion data if available
    if 'anomaly_motion_data' in proc_data:
        response['anomaly_motion_data'] = proc_data['anomaly_motion_data']
    
    return jsonify(response)


@app.route('/api/process/<processor_id>/stop', methods=['POST'])
def stop_processing(processor_id):
    """Stop processing"""
    if processor_id not in active_processors:
        return jsonify({'error': 'Processor not found'}), 404
    
    proc_data = active_processors[processor_id]
    processor = proc_data['processor']
    
    proc_data['running'] = False
    # processor.stop()  # Disabled - processor not used
    
    return jsonify({
        'status': 'stopped',
        'message': 'Processing stopped'
    })


@app.route('/api/process/<processor_id>/dismiss-anomaly', methods=['POST'])
def dismiss_anomaly(processor_id):
    """Dismiss/clear anomaly data to prevent popup from showing again"""
    if processor_id not in active_processors:
        return jsonify({'error': 'Processor not found'}), 404
    
    proc_data = active_processors[processor_id]
    
    # Clear anomaly_motion_data so popup doesn't show again
    if 'anomaly_motion_data' in proc_data:
        # Store in history if needed, but clear the active one
        if 'anomaly_history' not in proc_data:
            proc_data['anomaly_history'] = []
        proc_data['anomaly_history'].append(proc_data['anomaly_motion_data'])
        del proc_data['anomaly_motion_data']
    
    return jsonify({
        'status': 'success',
        'message': 'Anomaly dismissed'
    })


@app.route('/api/process/<processor_id>/set-threshold', methods=['POST'])
def set_anomaly_threshold(processor_id):
    """Update the anomaly detection threshold (boundary) - only allowed when not processing"""
    if processor_id not in active_processors:
        return jsonify({'error': 'Processor not found'}), 404
    
    proc_data = active_processors[processor_id]
    
    # Prevent threshold changes during processing
    if proc_data.get('running', False):
        return jsonify({'error': 'Cannot change threshold while processing is running. Please stop processing first.'}), 400
    
    data = request.json or {}
    threshold = data.get('threshold')
    
    if threshold is None:
        return jsonify({'error': 'Threshold value is required'}), 400
    
    try:
        threshold = float(threshold)
        if threshold <= 0:
            return jsonify({'error': 'Threshold must be a positive number'}), 400
    except (ValueError, TypeError):
        return jsonify({'error': 'Threshold must be a valid number'}), 400
    
    proc_data['anomaly_threshold_degrees'] = threshold
    
    print(f"[THRESHOLD] Updated anomaly threshold to {threshold}° for processor {processor_id}")
    
    return jsonify({
        'status': 'success',
        'message': f'Anomaly threshold updated to {threshold}°',
        'threshold': threshold
    })


@app.route('/api/process/<processor_id>/resume', methods=['POST'])
def resume_processing(processor_id):
    """Resume processing from where it stopped"""
    if processor_id not in active_processors:
        return jsonify({'error': 'Processor not found'}), 404
    
    proc_data = active_processors[processor_id]
    
    # Check if there's a resume index
    if 'resume_index' not in proc_data:
        return jsonify({'error': 'No resume point available. Anomaly must be detected first.'}), 400
    
    resume_index = proc_data['resume_index']
    bag_path = proc_data.get('bag_path')
    
    # Try to get bag path from cached readings or config
    if not bag_path:
        # Try to get from filePath in config
        config = proc_data.get('config', {})
        bag_path = config.get('filePath')
    
    if not bag_path:
        # Try to reconstruct from cached readings file path
        if 'cached_readings' in proc_data and len(proc_data['cached_readings']) > 0:
            # We can't get the path from readings, so we need to store it
            print(f"[RESUME] Warning: bag_path not found in proc_data. Available keys: {list(proc_data.keys())}")
            return jsonify({
                'error': 'Bag file path not found. Please restart processing.',
                'available_keys': list(proc_data.keys())
            }), 400
        else:
            return jsonify({'error': 'Bag file path not found and no cached readings available'}), 400
    
    # Clear the stop flag and resume
    proc_data['running'] = True
    proc_data['should_stop'] = False
    # Clear anomaly_motion_data so popup doesn't show again after resume
    if 'anomaly_motion_data' in proc_data:
        # Store in history if needed, but clear the active one
        if 'anomaly_history' not in proc_data:
            proc_data['anomaly_history'] = []
        proc_data['anomaly_history'].append(proc_data['anomaly_motion_data'])
        del proc_data['anomaly_motion_data']
    
    # Start processing from resume point in background thread
    thread = threading.Thread(
        target=process_ros_bag_resume,
        args=(processor_id, bag_path, proc_data.get('config', {}), resume_index)
    )
    thread.daemon = True
    thread.start()
    
    return jsonify({
        'status': 'resumed',
        'message': f'Processing resumed from index {resume_index}',
        'resume_index': resume_index
    })


@app.route('/api/process/<processor_id>/anomaly-data', methods=['GET'])
def get_anomaly_data(processor_id):
    """Get anomaly motion data for popup display"""
    if processor_id not in active_processors:
        return jsonify({'error': 'Processor not found'}), 404
    
    proc_data = active_processors[processor_id]
    
    if 'anomaly_motion_data' not in proc_data:
        return jsonify({'error': 'No anomaly data available'}), 404
    
    return jsonify({
        'status': 'success',
        'anomaly_data': proc_data['anomaly_motion_data']
    })


@app.route('/api/process/<processor_id>/download/all', methods=['GET'])
def download_all_data(processor_id):
    """Download all sensor data as CSV"""
    if processor_id not in active_processors:
        return jsonify({'error': 'Processor not found'}), 404
    
    proc_data = active_processors[processor_id]
    readings = proc_data.get('readings', [])
    
    if not readings:
        return jsonify({'error': 'No data available to download'}), 404
    
    # Convert readings to DataFrame
    data_rows = []
    for reading in readings:
        # Handle IMU data (can be tuple or list)
        imu_ang_vel = reading.imu_angular_velocity if hasattr(reading, 'imu_angular_velocity') and reading.imu_angular_velocity else None
        imu_lin_acc = reading.imu_linear_acceleration if hasattr(reading, 'imu_linear_acceleration') and reading.imu_linear_acceleration else None
        
        row = {
            'timestamp': reading.timestamp,
            'gps_lat': reading.gps_lat if hasattr(reading, 'gps_lat') and reading.gps_lat else None,
            'gps_lon': reading.gps_lon if hasattr(reading, 'gps_lon') and reading.gps_lon else None,
            'gps_altitude': reading.gps_altitude if hasattr(reading, 'gps_altitude') and reading.gps_altitude else None,
            'velocity': reading.velocity if hasattr(reading, 'velocity') and reading.velocity else None,
            'heading': reading.heading if hasattr(reading, 'heading') and reading.heading else None,
            'imu_angular_velocity_x': imu_ang_vel[0] if imu_ang_vel and len(imu_ang_vel) > 0 else None,
            'imu_angular_velocity_y': imu_ang_vel[1] if imu_ang_vel and len(imu_ang_vel) > 1 else None,
            'imu_angular_velocity_z': imu_ang_vel[2] if imu_ang_vel and len(imu_ang_vel) > 2 else None,
            'imu_linear_acceleration_x': imu_lin_acc[0] if imu_lin_acc and len(imu_lin_acc) > 0 else None,
            'imu_linear_acceleration_y': imu_lin_acc[1] if imu_lin_acc and len(imu_lin_acc) > 1 else None,
            'imu_linear_acceleration_z': imu_lin_acc[2] if imu_lin_acc and len(imu_lin_acc) > 2 else None,
            'wheel_odom_x': reading.wheel_odom_x if hasattr(reading, 'wheel_odom_x') and reading.wheel_odom_x else None,
            'wheel_odom_y': reading.wheel_odom_y if hasattr(reading, 'wheel_odom_y') and reading.wheel_odom_y else None,
            'wheel_odom_theta': reading.wheel_odom_theta if hasattr(reading, 'wheel_odom_theta') and reading.wheel_odom_theta else None,
            'cmd_vel_linear': reading.cmd_vel_linear if hasattr(reading, 'cmd_vel_linear') and reading.cmd_vel_linear else None,
            'cmd_vel_angular': reading.cmd_vel_angular if hasattr(reading, 'cmd_vel_angular') and reading.cmd_vel_angular else None,
        }
        data_rows.append(row)
    
    df = pd.DataFrame(data_rows)
    
    # Generate CSV
    csv_string = df.to_csv(index=False)
    
    # Create response
    response = Response(
        csv_string,
        mimetype='text/csv',
        headers={
            'Content-Disposition': f'attachment; filename=all_sensor_data_{processor_id}.csv'
        }
    )
    
    return response


@app.route('/api/process/<processor_id>/download/anomalies', methods=['GET'])
def download_anomalies(processor_id):
    """Download anomaly data as CSV"""
    if processor_id not in active_processors:
        return jsonify({'error': 'Processor not found'}), 404
    
    proc_data = active_processors[processor_id]
    anomalies = proc_data.get('anomalies', [])
    
    if not anomalies:
        return jsonify({'error': 'No anomalies available to download'}), 404
    
    # Convert anomalies to DataFrame
    data_rows = []
    for anomaly in anomalies:
        row = {
            'timestamp': anomaly.get('timestamp', None),
            'severity': anomaly.get('severity', None),
            'type': anomaly.get('type', None),
            'actual_lat': anomaly.get('position', {}).get('lat', None) if 'position' in anomaly else None,
            'actual_lon': anomaly.get('position', {}).get('lon', None) if 'position' in anomaly else None,
            'predicted_lat': anomaly.get('predicted_position', {}).get('lat', None) if 'predicted_position' in anomaly else None,
            'predicted_lon': anomaly.get('predicted_position', {}).get('lon', None) if 'predicted_position' in anomaly else None,
            'lat_diff_degrees': anomaly.get('lat_diff_degrees', None),
            'lon_diff_degrees': anomaly.get('lon_diff_degrees', None),
            'error_meters': anomaly.get('error_meters', None),
            'reasons': '; '.join(anomaly.get('reasons', [])) if 'reasons' in anomaly else None,
        }
        data_rows.append(row)
    
    df = pd.DataFrame(data_rows)
    
    # Generate CSV
    csv_string = df.to_csv(index=False)
    
    # Create response
    response = Response(
        csv_string,
        mimetype='text/csv',
        headers={
            'Content-Disposition': f'attachment; filename=anomalies_{processor_id}.csv'
        }
    )
    
    return response


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
    
    # Get detected anomalies (anomaly detection disabled)
    detected_anomalies = []  # No anomalies - feature disabled
    
    # Validate (anomaly detection disabled)
    # validation_result = poison_validator.validate_detection(injection_log, detected_anomalies)
    validation_result = {
        'validated': False,
        'message': 'Anomaly detection is disabled',
        'detected_count': 0,
        'expected_count': 0
    }
    
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
