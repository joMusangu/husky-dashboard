"""
Enhanced ROS Sensor Data Poisoning Detection Backend
Improved parsing for both ROS1 (.bag) and ROS2 (folder with metadata.yaml and .db3)
"""

import numpy as np
import pandas as pd
from datetime import datetime
from typing import Dict, List, Tuple, Optional, Any, TYPE_CHECKING
from dataclasses import dataclass, asdict
from collections import deque
import json
import struct
import threading
import queue
import time
from pathlib import Path
import sqlite3

# ROS bag parsing imports
try:
    from rosbags.rosbag1 import Reader as ROS1Reader
    ROS1_AVAILABLE = True
except ImportError:
    ROS1_AVAILABLE = False
    ROS1Reader = None

try:
    from rosbags.rosbag2 import Reader as ROS2Reader
    ROS2_AVAILABLE = True
except ImportError:
    ROS2_AVAILABLE = False
    ROS2Reader = None

try:
    from rosbags.serde import deserialize_cdr, cdr_to_ros1
    from rosbags.typesys import get_types_from_msg, Stores, get_typestore
    ROS2_SERDE_AVAILABLE = True
except ImportError:
    ROS2_SERDE_AVAILABLE = False
    deserialize_cdr = None
    cdr_to_ros1 = None

ROSBAGS_AVAILABLE = ROS1_AVAILABLE or ROS2_AVAILABLE
if ROSBAGS_AVAILABLE:
    if ROS1_AVAILABLE and ROS2_AVAILABLE:
        print("✓ rosbags library available (ROS1 + ROS2 support)")
    elif ROS1_AVAILABLE:
        print("✓ rosbags library available (ROS1 support only)")
    elif ROS2_AVAILABLE:
        print("✓ rosbags library available (ROS2 support only)")
else:
    print("⚠ Warning: rosbags not available")
    print("  Install with: pip install rosbags")

# CAN bus parsing
try:
    import can
    CAN_AVAILABLE = True
except ImportError:
    CAN_AVAILABLE = False
    print("⚠ Warning: python-can not available")


@dataclass
class SensorReading:
    """Container for multi-sensor readings at a timestamp"""
    timestamp: float
    gps_lat: float
    gps_lon: float
    gps_altitude: float = 0.0
    imu_orientation: Tuple[float, float, float, float] = (0, 0, 0, 1)
    imu_angular_velocity: Tuple[float, float, float] = (0, 0, 0)
    imu_linear_acceleration: Tuple[float, float, float] = (0, 0, 0)
    wheel_odom_x: float = 0.0
    wheel_odom_y: float = 0.0
    wheel_odom_theta: float = 0.0
    velocity: float = 0.0
    heading: float = 0.0
    cmd_vel_linear: float = 0.0
    cmd_vel_angular: float = 0.0
    
    def to_dict(self) -> Dict:
        """Convert to dictionary for JSON serialization"""
        return asdict(self)


class ROSBagParser:
    """Enhanced parser for ROS bag files (both ROS1 and ROS2 formats)"""
    
    def __init__(self):
        self.topic_map = {
            'gps': ['/gps/fix', '/fix', '/navsat/fix', '/gps/data', '/sensor/gps'],
            'imu': ['/imu/data', '/imu', '/imu/data_raw', '/sensor/imu'],
            'odom': ['/odom', '/odometry/filtered', '/husky_velocity_controller/odom', 
                    '/wheel_odom', '/odometry/wheel'],
            'cmd_vel': ['/cmd_vel', '/husky_velocity_controller/cmd_vel', '/mobile_base/commands/velocity']
        }
        
    def detect_bag_version(self, bag_path: str) -> int:
        """Detect if bag is ROS1 or ROS2"""
        path = Path(bag_path)
        
        if path.is_file() and path.suffix == '.bag':
            return 1
        elif path.is_dir():
            # Check for ROS2 bag folder structure
            if (path / 'metadata.yaml').exists():
                return 2
            # Check if any subdirectory is a ROS2 bag
            for subdir in path.iterdir():
                if subdir.is_dir() and (subdir / 'metadata.yaml').exists():
                    return 2
        
        raise ValueError(f"Unknown bag format: {bag_path}")
    
    def parse_ros1_bag(self, bag_path: str, topics: List[str] = None) -> List[SensorReading]:
        """Parse ROS1 bag file using rosbags library"""
        if not ROS1_AVAILABLE:
            raise ImportError("rosbags ROS1 support not available. Install with: pip install rosbags")
        
        print(f"[ROS1] Parsing bag: {bag_path}")
        
        readings = []
        sensor_cache = {}
        message_count = 0
        
        try:
            # Create typestore once (more efficient)
            from rosbags import typesys
            typestore = typesys.get_typestore(typesys.Stores.ROS1_NOETIC)
            
            with ROS1Reader(bag_path) as reader:
                # Get available topics
                connections = list(reader.connections)
                available_topics = {c.topic: c.msgtype for c in connections}
                
                print(f"[ROS1] Found {len(available_topics)} topics:")
                for topic, msgtype in list(available_topics.items())[:15]:
                    print(f"  • {topic} ({msgtype})")
                if len(available_topics) > 15:
                    print(f"  ... and {len(available_topics) - 15} more")
                
                # Build list of topics we care about
                topics_of_interest = (
                    self.topic_map['gps'] + 
                    self.topic_map['imu'] + 
                    self.topic_map['odom'] +
                    self.topic_map['cmd_vel']
                )
                
                # Find which topics exist in the bag
                relevant_topics = []
                for topic in available_topics.keys():
                    if any(interested in topic for interested in topics_of_interest):
                        relevant_topics.append(topic)
                
                print(f"[ROS1] Relevant sensor topics: {relevant_topics}")
                
                # Filter connections to only relevant topics
                relevant_connections = [c for c in connections if c.topic in relevant_topics]
                
                # Pre-load message definitions for relevant connections
                msgdefs = {}
                for conn in relevant_connections:
                    try:
                        msgdefs[conn.topic] = typestore.get_msgdef(conn.msgtype)
                    except Exception as e:
                        print(f"[ROS1] Warning: Could not load msgdef for {conn.topic}: {e}")
                
                # Iterate through messages from relevant topics only
                for connection, timestamp, rawdata in reader.messages(connections=relevant_connections):
                    message_count += 1
                    topic = connection.topic
                    
                    try:
                        # Get message definition (cached)
                        msgdef = msgdefs.get(topic)
                        if msgdef is None:
                            continue
                        
                        # deserialize_ros1 returns (message_object, position)
                        msg, _ = msgdef.deserialize_ros1(rawdata, 0, msgdef.cls, typestore)
                        t = timestamp * 1e-9  # Convert nanoseconds to seconds
                        
                        # Parse based on topic type
                        self._parse_message(topic, msg, t, sensor_cache)
                        
                        # Try to create complete reading
                        if self._has_complete_reading(sensor_cache):
                            reading = self._create_reading_from_cache(sensor_cache, t)
                            readings.append(reading)
                            
                            # Log progress every 100 readings
                            if len(readings) % 100 == 0:
                                print(f"[ROS1] Extracted {len(readings)} readings...")
                    
                    except Exception as e:
                        # Skip messages that can't be parsed
                        if message_count % 100 == 0:
                            print(f"[ROS1] Warning: Skipped message on {topic}: {e}")
                        continue
        
        except Exception as e:
            print(f"[ROS1] Error opening bag: {e}")
            raise
        
        print(f"[ROS1] ✓ Extracted {len(readings)} complete sensor readings from {message_count} messages")
        return readings
    
    def parse_ros2_bag(self, bag_path: str) -> List[SensorReading]:
        """Parse ROS2 bag folder (with metadata.yaml and .db3 files)"""
        if not ROSBAGS_AVAILABLE:
            raise ImportError("rosbags not available. Install with: pip install rosbags")
        
        print(f"[ROS2] Parsing bag folder: {bag_path}")
        
        bag_dir = Path(bag_path)
        
        # Verify it's a valid ROS2 bag
        if not (bag_dir / 'metadata.yaml').exists():
            raise ValueError(f"Not a valid ROS2 bag folder (missing metadata.yaml): {bag_path}")
        
        # Check for .db3 files
        db3_files = list(bag_dir.glob('*.db3'))
        if not db3_files:
            raise ValueError(f"No .db3 files found in ROS2 bag folder: {bag_path}")
        
        print(f"[ROS2] Found {len(db3_files)} .db3 files: {[f.name for f in db3_files]}")
        
        readings = []
        sensor_cache = {}
        message_count = 0
        
        try:
            with ROS2Reader(bag_path) as reader:
                # Get available topics
                connections = list(reader.connections)
                available_topics = {c.topic: c.msgtype for c in connections}
                
                print(f"[ROS2] Found {len(available_topics)} topics:")
                for topic, msgtype in list(available_topics.items())[:15]:
                    print(f"  • {topic} ({msgtype})")
                if len(available_topics) > 15:
                    print(f"  ... and {len(available_topics) - 15} more")
                
                # Build list of topics we care about
                topics_of_interest = (
                    self.topic_map['gps'] + 
                    self.topic_map['imu'] + 
                    self.topic_map['odom'] +
                    self.topic_map['cmd_vel']
                )
                
                # Find which topics exist in the bag
                relevant_topics = []
                for topic in available_topics.keys():
                    if any(interested in topic for interested in topics_of_interest):
                        relevant_topics.append(topic)
                
                print(f"[ROS2] Relevant sensor topics: {relevant_topics}")
                
                # Filter connections to only relevant topics
                relevant_connections = [c for c in connections if c.topic in relevant_topics]
                
                # Iterate through messages
                for connection, timestamp, rawdata in reader.messages(connections=relevant_connections):
                    message_count += 1
                    topic = connection.topic
                    
                    try:
                        # Deserialize message
                        msg = deserialize_cdr(rawdata, connection.msgtype)
                        t = timestamp * 1e-9  # Convert nanoseconds to seconds
                        
                        # Parse based on topic type
                        self._parse_message(topic, msg, t, sensor_cache)
                        
                        # Try to create complete reading
                        if self._has_complete_reading(sensor_cache):
                            reading = self._create_reading_from_cache(sensor_cache, t)
                            readings.append(reading)
                            
                            # Log progress every 100 readings
                            if len(readings) % 100 == 0:
                                print(f"[ROS2] Extracted {len(readings)} readings...")
                    
                    except Exception as e:
                        # Skip messages that can't be parsed
                        if message_count % 100 == 0:
                            print(f"[ROS2] Warning: Skipped message on {topic}: {e}")
                        continue
        
        except Exception as e:
            print(f"[ROS2] Error opening bag: {e}")
            import traceback
            traceback.print_exc()
            raise
        
        print(f"[ROS2] ✓ Extracted {len(readings)} complete sensor readings from {message_count} messages")
        return readings
    
    def _parse_message(self, topic: str, msg: Any, timestamp: float, sensor_cache: Dict):
        """Parse a message and update sensor cache"""
        
        # GPS data
        if any(gps_topic in topic for gps_topic in self.topic_map['gps']):
            if not sensor_cache.get('gps'):
                sensor_cache['gps'] = {}
            
            # Handle different GPS message types
            try:
                sensor_cache['gps'].update({
                    'timestamp': timestamp,
                    'gps_lat': float(msg.latitude),
                    'gps_lon': float(msg.longitude),
                    'gps_altitude': float(getattr(msg, 'altitude', 0.0))
                })
            except (AttributeError, TypeError) as e:
                pass
        
        # IMU data
        elif any(imu_topic in topic for imu_topic in self.topic_map['imu']):
            if not sensor_cache.get('imu'):
                sensor_cache['imu'] = {}
            
            try:
                orientation = msg.orientation
                angular_vel = msg.angular_velocity
                linear_acc = msg.linear_acceleration
                
                sensor_cache['imu'].update({
                    'timestamp': timestamp,
                    'imu_orientation': (
                        float(orientation.x), float(orientation.y),
                        float(orientation.z), float(orientation.w)
                    ),
                    'imu_angular_velocity': (
                        float(angular_vel.x), float(angular_vel.y),
                        float(angular_vel.z)
                    ),
                    'imu_linear_acceleration': (
                        float(linear_acc.x), float(linear_acc.y),
                        float(linear_acc.z)
                    )
                })
            except (AttributeError, TypeError) as e:
                pass
        
        # Odometry data
        elif any(odom_topic in topic for odom_topic in self.topic_map['odom']):
            if not sensor_cache.get('odom'):
                sensor_cache['odom'] = {}
            
            try:
                pose = msg.pose.pose
                twist = msg.twist.twist
                
                linear_vel = np.sqrt(
                    float(twist.linear.x)**2 + 
                    float(twist.linear.y)**2
                )
                
                sensor_cache['odom'].update({
                    'timestamp': timestamp,
                    'wheel_odom_x': float(pose.position.x),
                    'wheel_odom_y': float(pose.position.y),
                    'velocity': linear_vel,
                    'cmd_vel_linear': float(twist.linear.x),
                    'cmd_vel_angular': float(twist.angular.z)
                })
            except (AttributeError, TypeError) as e:
                pass
        
        # Command velocity data
        elif any(cmd_topic in topic for cmd_topic in self.topic_map['cmd_vel']):
            if not sensor_cache.get('cmd_vel'):
                sensor_cache['cmd_vel'] = {}
            
            try:
                sensor_cache['cmd_vel'].update({
                    'timestamp': timestamp,
                    'cmd_vel_linear': float(msg.linear.x),
                    'cmd_vel_angular': float(msg.angular.z)
                })
            except (AttributeError, TypeError) as e:
                pass
    
    def parse_bag(self, bag_path: str, topics: List[str] = None) -> List[SensorReading]:
        """Auto-detect and parse ROS bag file"""
        try:
            version = self.detect_bag_version(bag_path)
            
            if version == 1:
                return self.parse_ros1_bag(bag_path, topics)
            else:
                return self.parse_ros2_bag(bag_path)
        
        except Exception as e:
            print(f"[ERROR] Failed to parse bag: {e}")
            import traceback
            traceback.print_exc()
            raise
    
    def _has_complete_reading(self, cache: Dict) -> bool:
        """Check if we have enough data for a complete reading"""
        # At minimum, we need GPS data
        has_gps = 'gps' in cache and cache['gps'].get('gps_lat') is not None
        
        # Optionally check for other sensors
        return has_gps
    
    def _create_reading_from_cache(self, cache: Dict, timestamp: float) -> SensorReading:
        """Create SensorReading from cached sensor data"""
        reading = SensorReading(
            timestamp=timestamp,
            gps_lat=cache.get('gps', {}).get('gps_lat', 0.0),
            gps_lon=cache.get('gps', {}).get('gps_lon', 0.0),
            gps_altitude=cache.get('gps', {}).get('gps_altitude', 0.0),
            imu_orientation=cache.get('imu', {}).get('imu_orientation', (0, 0, 0, 1)),
            imu_angular_velocity=cache.get('imu', {}).get('imu_angular_velocity', (0, 0, 0)),
            imu_linear_acceleration=cache.get('imu', {}).get('imu_linear_acceleration', (0, 0, 0)),
            wheel_odom_x=cache.get('odom', {}).get('wheel_odom_x', 0.0),
            wheel_odom_y=cache.get('odom', {}).get('wheel_odom_y', 0.0),
            velocity=cache.get('odom', {}).get('velocity', 0.0),
            cmd_vel_linear=cache.get('odom', {}).get('cmd_vel_linear', 0.0),
            cmd_vel_angular=cache.get('odom', {}).get('cmd_vel_angular', 0.0)
        )
        return reading


class CANBusParser:
    """Parse CAN bus logs for vehicle control data"""
    
    def __init__(self):
        self.message_definitions = {
            0x100: self._parse_motor_command,
            0x200: self._parse_steering_command,
            0x300: self._parse_status,
        }
    
    def parse_can_log(self, log_path: str) -> List[Dict]:
        """Parse CAN log file (supports .asc, .log, .blf formats)"""
        if not CAN_AVAILABLE:
            raise ImportError("python-can not available. Install with: pip install python-can")
        
        print(f"[CAN] Parsing log: {log_path}")
        
        messages = []
        ext = Path(log_path).suffix.lower()
        
        try:
            if ext == '.asc':
                log_reader = can.ASCReader(log_path)
            elif ext == '.blf':
                log_reader = can.BLFReader(log_path)
            elif ext in ['.log', '.txt']:
                log_reader = can.LogReader(log_path)
            else:
                raise ValueError(f"Unsupported CAN log format: {ext}")
            
            for msg in log_reader:
                parsed = self._parse_message(msg)
                if parsed:
                    messages.append(parsed)
            
            print(f"[CAN] ✓ Parsed {len(messages)} CAN messages")
        
        except Exception as e:
            print(f"[CAN] Error: {e}")
            raise
        
        return messages
    
    def _parse_message(self, msg) -> Optional[Dict]:
        """Parse individual CAN message"""
        parser = self.message_definitions.get(msg.arbitration_id)
        if parser:
            return parser(msg)
        return None
    
    def _parse_motor_command(self, msg) -> Dict:
        """Parse motor command message"""
        data = struct.unpack('<HH', msg.data[:4])
        return {
            'timestamp': msg.timestamp,
            'type': 'motor_command',
            'left_motor': data[0],
            'right_motor': data[1]
        }
    
    def _parse_steering_command(self, msg) -> Dict:
        """Parse steering command message"""
        angle = struct.unpack('<h', msg.data[:2])[0]
        return {
            'timestamp': msg.timestamp,
            'type': 'steering',
            'angle': angle / 100.0
        }
    
    def _parse_status(self, msg) -> Dict:
        """Parse status message"""
        return {
            'timestamp': msg.timestamp,
            'type': 'status',
            'status_code': msg.data[0]
        }


class TemporalConsistencyChecker:
    """Validates sensor readings based on temporal patterns"""
    
    def __init__(self, window_size: int = 10):
        self.window_size = window_size
        self.history = deque(maxlen=window_size)
        
    def check_velocity_consistency(self, current: SensorReading, 
                                   max_acceleration: float = 5.0) -> Tuple[bool, str]:
        """Check if velocity changes are physically plausible"""
        if len(self.history) < 2:
            return True, ""
            
        prev = self.history[-1]
        dt = current.timestamp - prev.timestamp
        
        if dt <= 0:
            return False, "Non-positive time delta"
            
        acceleration = abs(current.velocity - prev.velocity) / dt
        
        if acceleration > max_acceleration:
            return False, f"Implausible acceleration: {acceleration:.2f} m/s²"
        
        return True, ""
    
    def check_position_jump(self, current: SensorReading, 
                           max_jump_distance: float = 10.0) -> Tuple[bool, str]:
        """Detect unrealistic position jumps in GPS"""
        if len(self.history) < 1:
            return True, ""
            
        prev = self.history[-1]
        dt = current.timestamp - prev.timestamp
        
        if dt <= 0:
            return True, ""
        
        # Calculate distance using haversine formula
        R = 6371000  # Earth radius in meters
        lat1, lon1 = np.radians(prev.gps_lat), np.radians(prev.gps_lon)
        lat2, lon2 = np.radians(current.gps_lat), np.radians(current.gps_lon)
        
        dlat = lat2 - lat1
        dlon = lon2 - lon1
        
        a = np.sin(dlat/2)**2 + np.cos(lat1) * np.cos(lat2) * np.sin(dlon/2)**2
        c = 2 * np.arctan2(np.sqrt(a), np.sqrt(1-a))
        distance = R * c
        
        # Check if distance is reasonable given time and max velocity
        max_distance = 20.0 * dt  # Assuming max 20 m/s for UGV
        
        if distance > max_distance or distance > max_jump_distance:
            return False, f"GPS jump: {distance:.2f}m in {dt:.2f}s"
        
        return True, ""
    
    def add_reading(self, reading: SensorReading):
        """Add reading to history"""
        self.history.append(reading)


class CrossSensorValidator:
    """Validates consistency across different sensor modalities"""
    
    def __init__(self):
        self.gps_odom_threshold = 5.0  # meters
        
    def validate_gps_vs_odometry(self, current: SensorReading, 
                                 previous: SensorReading) -> Tuple[bool, str]:
        """Compare GPS displacement with wheel odometry displacement"""
        
        # GPS displacement using haversine
        R = 6371000
        lat1, lon1 = np.radians(previous.gps_lat), np.radians(previous.gps_lon)
        lat2, lon2 = np.radians(current.gps_lat), np.radians(current.gps_lon)
        
        dlat = lat2 - lat1
        dlon = lon2 - lon1
        
        a = np.sin(dlat/2)**2 + np.cos(lat1) * np.cos(lat2) * np.sin(dlon/2)**2
        c = 2 * np.arctan2(np.sqrt(a), np.sqrt(1-a))
        gps_displacement = R * c
        
        # Odometry displacement
        dx = current.wheel_odom_x - previous.wheel_odom_x
        dy = current.wheel_odom_y - previous.wheel_odom_y
        odom_displacement = np.sqrt(dx**2 + dy**2)
        
        # Check if displacements agree within threshold
        difference = abs(gps_displacement - odom_displacement)
        
        if difference > self.gps_odom_threshold:
            return False, f"GPS-Odom mismatch: {difference:.2f}m"
        
        return True, ""


class TrustScoreManager:
    """Manages dynamic trust scores for each sensor"""
    
    def __init__(self):
        self.scores = {
            'gps': 1.0,
            'imu': 1.0,
            'odometry': 1.0
        }
        self.min_score = 0.1
        self.max_score = 1.0
        
    def penalize(self, sensor: str, severity: float = 0.1):
        """Reduce trust score for a sensor"""
        if sensor in self.scores:
            self.scores[sensor] = max(self.min_score, 
                                     self.scores[sensor] * (1 - severity))
    
    def reward(self, sensor: str, amount: float = 0.05):
        """Increase trust score for consistent sensor"""
        if sensor in self.scores:
            self.scores[sensor] = min(self.max_score, 
                                     self.scores[sensor] + amount)
    
    def get_scores(self) -> Dict[str, float]:
        """Return current trust scores"""
        return self.scores.copy()


class AnomalyDetector:
    """Main anomaly detection engine with real-time capabilities"""
    
    def __init__(self, prediction_window: int = 5):
        self.prediction_window = prediction_window
        self.temporal_checker = TemporalConsistencyChecker(window_size=10)
        self.cross_validator = CrossSensorValidator()
        self.trust_manager = TrustScoreManager()
        self.anomaly_log = []
        self.history = []
        
    def predict_next_position(self, history: List[SensorReading]) -> Tuple[float, float]:
        """Predict next GPS position using linear regression"""
        if len(history) < 2:
            return history[-1].gps_lat, history[-1].gps_lon
        
        recent = history[-self.prediction_window:]
        
        lats = np.array([r.gps_lat for r in recent])
        lons = np.array([r.gps_lon for r in recent])
        times = np.array([r.timestamp for r in recent])
        
        # Linear regression
        if len(times) > 1 and times[-1] != times[0]:
            lat_velocity = (lats[-1] - lats[0]) / (times[-1] - times[0])
            lon_velocity = (lons[-1] - lons[0]) / (times[-1] - times[0])
            
            dt = 1.0
            predicted_lat = lats[-1] + lat_velocity * dt
            predicted_lon = lons[-1] + lon_velocity * dt
        else:
            predicted_lat, predicted_lon = lats[-1], lons[-1]
        
        return predicted_lat, predicted_lon
    
    def detect_anomaly(self, current: SensorReading) -> Optional[Dict]:
        """Main detection function - returns anomaly report"""
        
        anomaly_detected = False
        anomaly_reasons = []
        severity = 0.0
        
        # Temporal consistency checks
        vel_ok, vel_msg = self.temporal_checker.check_velocity_consistency(current)
        if not vel_ok:
            anomaly_detected = True
            anomaly_reasons.append(vel_msg)
            severity += 0.3
            self.trust_manager.penalize('gps', 0.15)
        
        pos_ok, pos_msg = self.temporal_checker.check_position_jump(current)
        if not pos_ok:
            anomaly_detected = True
            anomaly_reasons.append(pos_msg)
            severity += 0.4
            self.trust_manager.penalize('gps', 0.2)
        
        # Cross-sensor validation
        if len(self.history) >= 1:
            prev = self.history[-1]
            cross_ok, cross_msg = self.cross_validator.validate_gps_vs_odometry(
                current, prev
            )
            if not cross_ok:
                anomaly_detected = True
                anomaly_reasons.append(cross_msg)
                severity += 0.3
                self.trust_manager.penalize('gps', 0.15)
        
        # Prediction-based detection
        if len(self.history) >= self.prediction_window:
            pred_lat, pred_lon = self.predict_next_position(self.history)
            
            # Calculate prediction error
            R = 6371000
            lat_error = R * np.radians(current.gps_lat - pred_lat)
            lon_error = (R * np.radians(current.gps_lon - pred_lon) * 
                        np.cos(np.radians(current.gps_lat)))
            prediction_error = np.sqrt(lat_error**2 + lon_error**2)
            
            threshold = 2.0 + (self.prediction_window * 0.5)
            
            if prediction_error > threshold:
                anomaly_detected = True
                anomaly_reasons.append(f"Prediction error: {prediction_error:.2f}m")
                severity += min(0.5, prediction_error / 20.0)
                self.trust_manager.penalize('gps', 0.1)
        
        # Update history
        self.temporal_checker.add_reading(current)
        self.history.append(current)
        
        # Generate anomaly report
        if anomaly_detected:
            anomaly_report = {
                'timestamp': current.timestamp,
                'severity': min(1.0, severity),
                'reasons': anomaly_reasons,
                'trust_scores': self.trust_manager.get_scores(),
                'position': {
                    'lat': current.gps_lat,
                    'lon': current.gps_lon
                }
            }
            self.anomaly_log.append(anomaly_report)
            return anomaly_report
        else:
            self.trust_manager.reward('gps')
            return None
    
    def export_results(self, output_path: str):
        """Export anomaly detection results to JSON"""
        results = {
            'total_readings': len(self.history),
            'total_anomalies': len(self.anomaly_log),
            'detection_rate': len(self.anomaly_log) / len(self.history) if self.history else 0,
            'final_trust_scores': self.trust_manager.get_scores(),
            'anomalies': self.anomaly_log
        }
        
        with open(output_path, 'w') as f:
            json.dump(results, f, indent=2)
        
        print(f"[EXPORT] ✓ Results exported to {output_path}")


class RealtimeProcessor:
    """Real-time data processor with threading support"""
    
    def __init__(self, detector: AnomalyDetector):
        self.detector = detector
        self.data_queue = queue.Queue()
        self.result_queue = queue.Queue()
        self.running = False
        self.processing_thread = None
        
    def start(self):
        """Start real-time processing thread"""
        self.running = True
        self.processing_thread = threading.Thread(target=self._process_loop)
        self.processing_thread.daemon = True
        self.processing_thread.start()
    
    def stop(self):
        """Stop processing thread"""
        self.running = False
        if self.processing_thread:
            self.processing_thread.join(timeout=2.0)
    
    def add_reading(self, reading: SensorReading):
        """Add sensor reading to processing queue"""
        self.data_queue.put(reading)
    
    def get_results(self) -> List[Dict]:
        """Get all available results"""
        results = []
        while not self.result_queue.empty():
            try:
                results.append(self.result_queue.get_nowait())
            except queue.Empty:
                break
        return results
    
    def _process_loop(self):
        """Main processing loop"""
        while self.running:
            try:
                reading = self.data_queue.get(timeout=0.1)
                
                # Process reading
                start_time = time.time()
                anomaly = self.detector.detect_anomaly(reading)
                processing_time = time.time() - start_time
                
                # Prepare result
                result = {
                    'timestamp': reading.timestamp,
                    'processing_time': processing_time,
                    'anomaly': anomaly,
                    'trust_scores': self.detector.trust_manager.get_scores()
                }
                
                self.result_queue.put(result)
                
            except queue.Empty:
                continue


# Main execution example
if __name__ == "__main__":
    print("=" * 70)
    print("ROS Sensor Data Poisoning Detection System - Complete Backend")
    print("=" * 70)
    
    # Example: Generate synthetic data
    print("\n[Test] Generating synthetic data...")
    
    from datetime import datetime
    readings = []
    lat, lon = 33.5779, -101.8552
    
    for i in range(100):
        t = i * 0.1
        lat += 0.00001 * (1 + 0.1 * np.sin(t))
        lon += 0.00001 * (1 + 0.1 * np.cos(t))
        
        reading = SensorReading(
            timestamp=t,
            gps_lat=lat,
            gps_lon=lon,
            velocity=2.0 + 0.5 * np.sin(t)
        )
        readings.append(reading)
    
    print(f"✓ Generated {len(readings)} synthetic readings")
    
    # Test anomaly detection
    print("\n[Test] Running anomaly detection...")
    detector = AnomalyDetector(prediction_window=5)
    processor = RealtimeProcessor(detector)
    
    processor.start()
    
    # Process readings
    for i, reading in enumerate(readings):
        # Inject poisoning
        if 40 <= i < 60:
            reading.gps_lat += 0.001
            reading.gps_lon += 0.001
        
        processor.add_reading(reading)
        time.sleep(0.01)
    
    # Wait for processing
    time.sleep(0.5)
    processor.stop()
    
    # Get results
    results = processor.get_results()
    anomalies = [r for r in results if r['anomaly'] is not None]
    
    print(f"✓ Processed {len(results)} readings")
    print(f"✓ Detected {len(anomalies)} anomalies")
    print(f"✓ Detection rate: {len(anomalies)/len(results)*100:.1f}%")
    
    print("\n" + "=" * 70)
    print("Backend test complete!")
    print("=" * 70)