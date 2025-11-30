"""
Flask API Server for ROS Sensor Data Poisoning Detection System
Connects the backend.py functionality to the frontend
"""

from flask import Flask, request, jsonify, send_from_directory
from flask_cors import CORS
import os
import tempfile
import threading
from pathlib import Path
import json

# Import backend classes
from backend import (
    ROSBagParser, CANBusParser, AnomalyDetector, 
    RealtimeProcessor, SensorReading
)
import numpy as np

app = Flask(__name__, static_folder='.')
CORS(app)  # Enable CORS for frontend requests

# Global state
active_processors = {}
processor_counter = 0


@app.route('/')
def index():
    """Serve the main HTML file"""
    return send_from_directory('.', 'index.html')


@app.route('/api/health', methods=['GET'])
def health():
    """Health check endpoint"""
    return jsonify({
        'status': 'healthy',
        'message': 'ROS Sensor Data Poisoning Detection API is running'
    })


@app.route('/api/process/start', methods=['POST'])
def start_processing():
    """Start processing ROS bag file or generate synthetic data"""
    global processor_counter
    
    data = request.json or {}
    config = data.get('config', {})
    use_synthetic = data.get('useSynthetic', True)
    
    processor_id = f"processor_{processor_counter}"
    processor_counter += 1
    
    # Create detector and processor
    prediction_window = config.get('predictionWindow', 5)
    detector = AnomalyDetector(prediction_window=prediction_window)
    processor = RealtimeProcessor(detector)
    
    active_processors[processor_id] = {
        'processor': processor,
        'detector': detector,
        'readings': [],
        'results': [],
        'config': config
    }
    
    processor.start()
    
    # Start processing in background thread
    if use_synthetic:
        thread = threading.Thread(
            target=process_synthetic_data,
            args=(processor_id, config)
        )
    else:
        # Handle file upload if provided
        file_path = data.get('filePath')
        if file_path and Path(file_path).exists():
            thread = threading.Thread(
                target=process_ros_bag,
                args=(processor_id, file_path, config)
            )
        else:
            thread = threading.Thread(
                target=process_synthetic_data,
                args=(processor_id, config)
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
    if processor_id not in active_processors:
        return
    
    processor = active_processors[processor_id]['processor']
    detector = active_processors[processor_id]['detector']
    
    # Generate synthetic data similar to backend.py
    readings = []
    lat, lon = 33.5779, -101.8552
    
    for i in range(200):
        if processor_id not in active_processors:
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
        
        # Inject poisoning
        if 80 <= i < 120:
            reading.gps_lat += 0.0005
            reading.gps_lon += 0.0005
        
        readings.append(reading)
        processor.add_reading(reading)
        
        # Store reading
        active_processors[processor_id]['readings'].append(reading)
        
        # Get results
        results = processor.get_results()
        active_processors[processor_id]['results'].extend(results)
        
        import time
        time.sleep(0.05)  # Simulate real-time
    
    # Wait for processing to complete
    import time
    time.sleep(0.5)
    processor.stop()


def process_ros_bag(processor_id, bag_path, config):
    """Process actual ROS bag file"""
    if processor_id not in active_processors:
        return
    
    processor = active_processors[processor_id]['processor']
    parser = ROSBagParser()
    
    try:
        readings = parser.parse_bag(bag_path)
        
        for reading in readings:
            if processor_id not in active_processors:
                break
                
            processor.add_reading(reading)
            active_processors[processor_id]['readings'].append(reading)
            
            # Get results periodically
            results = processor.get_results()
            active_processors[processor_id]['results'].extend(results)
            
            import time
            time.sleep(0.01)
        
        import time
        time.sleep(0.5)
        processor.stop()
        
    except Exception as e:
        print(f"Error processing ROS bag: {e}")
        processor.stop()


@app.route('/api/process/<processor_id>/status', methods=['GET'])
def get_status(processor_id):
    """Get current processing status"""
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
    for i, reading in enumerate(readings):
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
        avg_processing_time = sum(processing_times) / len(processing_times) * 1000  # Convert to ms
    
    return jsonify({
        'isProcessing': processor.running,
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
    })


@app.route('/api/process/<processor_id>/stop', methods=['POST'])
def stop_processing(processor_id):
    """Stop processing"""
    if processor_id not in active_processors:
        return jsonify({'error': 'Processor not found'}), 404
    
    proc_data = active_processors[processor_id]
    processor = proc_data['processor']
    
    processor.stop()
    del active_processors[processor_id]
    
    return jsonify({
        'status': 'stopped',
        'message': 'Processing stopped'
    })


@app.route('/api/upload', methods=['POST'])
def upload_file():
    """Handle ROS bag file upload"""
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
    
    return jsonify({
        'filePath': str(file_path),
        'filename': file.filename,
        'message': 'File uploaded successfully'
    })


if __name__ == '__main__':
    # Create uploads directory
    Path('uploads').mkdir(exist_ok=True)
    
    print("=" * 70)
    print("ROS Sensor Data Poisoning Detection - Flask API Server")
    print("=" * 70)
    print("\nStarting server on http://localhost:5000")
    print("Frontend will be available at http://localhost:5000")
    print("\nPress Ctrl+C to stop the server")
    print("=" * 70)
    
    app.run(debug=True, host='0.0.0.0', port=5000)

