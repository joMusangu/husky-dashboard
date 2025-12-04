"""
Data Poisoning Injection System
Allows controlled injection of various poisoning attacks for testing
"""

import numpy as np
import copy
from dataclasses import dataclass
from typing import List, Dict, Optional, Tuple
from enum import Enum


class PoisonType(Enum):
    """Types of poisoning attacks"""
    GPS_JUMP = "gps_jump"              # Sudden position jump
    GPS_DRIFT = "gps_drift"            # Gradual position drift
    GPS_FREEZE = "gps_freeze"          # GPS readings freeze
    IMU_NOISE = "imu_noise"            # Add noise to IMU
    IMU_BIAS = "imu_bias"              # Add bias to IMU
    ODOM_SCALING = "odom_scaling"      # Scale odometry values
    VELOCITY_SPIKE = "velocity_spike"  # Sudden velocity change
    ALTITUDE_JUMP = "altitude_jump"    # Altitude anomaly
    SENSOR_DROPOUT = "sensor_dropout"  # Sensor drops out
    REPLAY_ATTACK = "replay_attack"    # Replay old data


@dataclass
class PoisonConfig:
    """Configuration for poisoning injection"""
    poison_type: PoisonType
    start_time: float = 0.0           # When to start (seconds)
    duration: float = 1.0             # How long (seconds)
    intensity: float = 1.0            # Severity multiplier (0-1)
    target_sensor: str = "gps"        # Which sensor to poison
    
    # Type-specific parameters
    jump_distance: float = 50.0       # meters for GPS jump
    drift_rate: float = 5.0           # m/s for GPS drift
    noise_stddev: float = 0.1         # for IMU noise
    bias_value: float = 0.5           # for IMU bias
    scale_factor: float = 2.0         # for odometry scaling


class PoisonInjector:
    """Injects poisoning attacks into sensor data"""
    
    def __init__(self):
        self.active_poisons: List[PoisonConfig] = []
        self.injection_log: List[Dict] = []
        
    def add_poison(self, config: PoisonConfig):
        """Add a poisoning attack to inject"""
        self.active_poisons.append(config)
        print(f"[POISON] Added {config.poison_type.value}: "
              f"t={config.start_time:.1f}s, duration={config.duration:.1f}s, "
              f"intensity={config.intensity:.1f}")
    
    def clear_poisons(self):
        """Remove all active poisoning configurations"""
        self.active_poisons.clear()
        print("[POISON] Cleared all poisoning configurations")
    
    def inject(self, reading, current_time: float):
        """
        Inject poisoning into a sensor reading
        Returns: (modified_reading, is_poisoned, poison_info)
        """
        modified = copy.deepcopy(reading)
        is_poisoned = False
        applied_poisons = []
        
        for poison in self.active_poisons:
            # Check if poison should be active now
            if poison.start_time <= current_time < (poison.start_time + poison.duration):
                # Calculate time within poison window
                poison_progress = (current_time - poison.start_time) / poison.duration
                
                # Apply the poisoning
                if poison.poison_type == PoisonType.GPS_JUMP:
                    modified = self._inject_gps_jump(modified, poison, poison_progress)
                    is_poisoned = True
                    applied_poisons.append("GPS Jump")
                    
                elif poison.poison_type == PoisonType.GPS_DRIFT:
                    modified = self._inject_gps_drift(modified, poison, poison_progress)
                    is_poisoned = True
                    applied_poisons.append("GPS Drift")
                    
                elif poison.poison_type == PoisonType.GPS_FREEZE:
                    modified = self._inject_gps_freeze(modified, poison, poison_progress)
                    is_poisoned = True
                    applied_poisons.append("GPS Freeze")
                    
                elif poison.poison_type == PoisonType.IMU_NOISE:
                    modified = self._inject_imu_noise(modified, poison, poison_progress)
                    is_poisoned = True
                    applied_poisons.append("IMU Noise")
                    
                elif poison.poison_type == PoisonType.IMU_BIAS:
                    modified = self._inject_imu_bias(modified, poison, poison_progress)
                    is_poisoned = True
                    applied_poisons.append("IMU Bias")
                    
                elif poison.poison_type == PoisonType.ODOM_SCALING:
                    modified = self._inject_odom_scaling(modified, poison, poison_progress)
                    is_poisoned = True
                    applied_poisons.append("Odometry Scaling")
                    
                elif poison.poison_type == PoisonType.VELOCITY_SPIKE:
                    modified = self._inject_velocity_spike(modified, poison, poison_progress)
                    is_poisoned = True
                    applied_poisons.append("Velocity Spike")
                    
                elif poison.poison_type == PoisonType.ALTITUDE_JUMP:
                    modified = self._inject_altitude_jump(modified, poison, poison_progress)
                    is_poisoned = True
                    applied_poisons.append("Altitude Jump")
        
        # Log injection
        if is_poisoned:
            self.injection_log.append({
                'timestamp': current_time,
                'poisons': applied_poisons,
                'intensity': max(p.intensity for p in self.active_poisons 
                               if p.start_time <= current_time < (p.start_time + p.duration))
            })
        
        poison_info = {
            'is_poisoned': is_poisoned,
            'applied_poisons': applied_poisons,
            'active_count': len([p for p in self.active_poisons 
                               if p.start_time <= current_time < (p.start_time + p.duration)])
        }
        
        return modified, is_poisoned, poison_info
    
    def _inject_gps_jump(self, reading, config: PoisonConfig, progress: float):
        """Inject sudden GPS position jump"""
        if progress < 0.1:  # Apply jump at start
            # Random direction jump
            angle = np.random.uniform(0, 2 * np.pi)
            distance = config.jump_distance * config.intensity
            
            # Convert to lat/lon offset (approximate)
            lat_offset = (distance / 111320.0) * np.cos(angle)  # 1 degree lat ≈ 111.32 km
            lon_offset = (distance / (111320.0 * np.cos(np.radians(reading.gps_lat)))) * np.sin(angle)
            
            reading.gps_lat += lat_offset
            reading.gps_lon += lon_offset
        
        return reading
    
    def _inject_gps_drift(self, reading, config: PoisonConfig, progress: float):
        """Inject gradual GPS drift"""
        # Drift accumulates over time
        elapsed = progress * config.duration
        drift_distance = config.drift_rate * elapsed * config.intensity
        
        # Constant drift direction (stored in first call)
        if not hasattr(self, '_drift_angle'):
            self._drift_angle = np.random.uniform(0, 2 * np.pi)
        
        lat_offset = (drift_distance / 111320.0) * np.cos(self._drift_angle)
        lon_offset = (drift_distance / (111320.0 * np.cos(np.radians(reading.gps_lat)))) * np.sin(self._drift_angle)
        
        reading.gps_lat += lat_offset
        reading.gps_lon += lon_offset
        
        return reading
    
    def _inject_gps_freeze(self, reading, config: PoisonConfig, progress: float):
        """Freeze GPS readings"""
        if not hasattr(self, '_frozen_gps'):
            self._frozen_gps = (reading.gps_lat, reading.gps_lon, reading.gps_altitude)
        
        reading.gps_lat, reading.gps_lon, reading.gps_altitude = self._frozen_gps
        return reading
    
    def _inject_imu_noise(self, reading, config: PoisonConfig, progress: float):
        """Add noise to IMU readings"""
        noise_scale = config.noise_stddev * config.intensity
        
        # Add noise to angular velocity
        noisy_vel = tuple(v + np.random.normal(0, noise_scale) 
                         for v in reading.imu_angular_velocity)
        reading.imu_angular_velocity = noisy_vel
        
        # Add noise to linear acceleration
        noisy_accel = tuple(a + np.random.normal(0, noise_scale * 10) 
                           for a in reading.imu_linear_acceleration)
        reading.imu_linear_acceleration = noisy_accel
        
        return reading
    
    def _inject_imu_bias(self, reading, config: PoisonConfig, progress: float):
        """Add bias to IMU readings"""
        bias = config.bias_value * config.intensity
        
        # Add constant bias
        biased_vel = tuple(v + bias for v in reading.imu_angular_velocity)
        reading.imu_angular_velocity = biased_vel
        
        return reading
    
    def _inject_odom_scaling(self, reading, config: PoisonConfig, progress: float):
        """Scale odometry values"""
        scale = 1.0 + (config.scale_factor - 1.0) * config.intensity
        
        # Scale odometry positions and velocity
        reading.wheel_odom_x *= scale
        reading.wheel_odom_y *= scale
        reading.velocity *= scale
        reading.cmd_vel_linear *= scale
        reading.cmd_vel_angular *= scale
        
        return reading
    
    def _inject_velocity_spike(self, reading, config: PoisonConfig, progress: float):
        """Inject sudden velocity spike"""
        if progress < 0.2:  # Spike at start
            spike_magnitude = 10.0 * config.intensity
            
            # Add spike to velocity
            reading.velocity += spike_magnitude
            reading.cmd_vel_linear += spike_magnitude
        
        return reading
    
    def _inject_altitude_jump(self, reading, config: PoisonConfig, progress: float):
        """Inject altitude jump"""
        if progress < 0.1:  # Jump at start
            reading.gps_altitude += config.jump_distance * config.intensity
        
        return reading
    
    def get_injection_stats(self) -> Dict:
        """Get statistics about injected poisons"""
        return {
            'total_injections': len(self.injection_log),
            'active_poisons': len(self.active_poisons),
            'poison_types': list(set(p.poison_type.value for p in self.active_poisons)),
            'injection_history': self.injection_log[-10:]  # Last 10
        }


class PoisonValidator:
    """Validates that anomaly detection is working correctly"""
    
    def __init__(self):
        self.validation_results = []
        
    def validate_detection(self, injected_poisons: List[Dict], detected_anomalies: List[Dict]) -> Dict:
        """
        Validate that injected poisons were detected as anomalies
        Returns validation metrics
        """
        # Extract timestamps
        injected_times = set(p['timestamp'] for p in injected_poisons)
        detected_times = set(a['timestamp'] for a in detected_anomalies)
        
        # Calculate metrics with tolerance window (±0.5s)
        tolerance = 0.5
        
        true_positives = 0  # Correctly detected poisons
        false_negatives = 0  # Missed poisons
        false_positives = 0  # Detected non-poisons as anomalies
        
        # Check each injection
        for inj_time in injected_times:
            detected = any(abs(det_time - inj_time) < tolerance for det_time in detected_times)
            if detected:
                true_positives += 1
            else:
                false_negatives += 1
        
        # Check for false positives (detections not near injections)
        for det_time in detected_times:
            near_injection = any(abs(inj_time - det_time) < tolerance for inj_time in injected_times)
            if not near_injection:
                false_positives += 1
        
        # Calculate rates
        total_injections = len(injected_times)
        total_detections = len(detected_times)
        
        detection_rate = (true_positives / total_injections * 100) if total_injections > 0 else 0
        false_positive_rate = (false_positives / total_detections * 100) if total_detections > 0 else 0
        
        precision = (true_positives / (true_positives + false_positives)) if (true_positives + false_positives) > 0 else 0
        recall = (true_positives / (true_positives + false_negatives)) if (true_positives + false_negatives) > 0 else 0
        f1_score = (2 * precision * recall / (precision + recall)) if (precision + recall) > 0 else 0
        
        result = {
            'true_positives': true_positives,
            'false_negatives': false_negatives,
            'false_positives': false_positives,
            'detection_rate': detection_rate,
            'false_positive_rate': false_positive_rate,
            'precision': precision,
            'recall': recall,
            'f1_score': f1_score,
            'total_injections': total_injections,
            'total_detections': total_detections
        }
        
        self.validation_results.append(result)
        
        return result
    
    def get_validation_summary(self) -> Dict:
        """Get summary of all validation runs"""
        if not self.validation_results:
            return {'status': 'No validations performed'}
        
        # Average metrics across all validations
        avg_detection_rate = np.mean([r['detection_rate'] for r in self.validation_results])
        avg_precision = np.mean([r['precision'] for r in self.validation_results])
        avg_recall = np.mean([r['recall'] for r in self.validation_results])
        avg_f1 = np.mean([r['f1_score'] for r in self.validation_results])
        
        return {
            'total_validations': len(self.validation_results),
            'avg_detection_rate': avg_detection_rate,
            'avg_precision': avg_precision,
            'avg_recall': avg_recall,
            'avg_f1_score': avg_f1,
            'latest_result': self.validation_results[-1]
        }


# Preset attack scenarios for quick testing
PRESET_ATTACKS = {
    'quick_test': [
        PoisonConfig(PoisonType.GPS_JUMP, start_time=2.0, duration=0.5, intensity=1.0, jump_distance=50.0),
        PoisonConfig(PoisonType.GPS_JUMP, start_time=5.0, duration=0.5, intensity=1.0, jump_distance=75.0),
    ],
    'sustained_drift': [
        PoisonConfig(PoisonType.GPS_DRIFT, start_time=3.0, duration=5.0, intensity=0.8, drift_rate=5.0),
    ],
    'sensor_freeze': [
        PoisonConfig(PoisonType.GPS_FREEZE, start_time=4.0, duration=3.0, intensity=1.0),
    ],
    'imu_attack': [
        PoisonConfig(PoisonType.IMU_NOISE, start_time=2.0, duration=4.0, intensity=1.0, noise_stddev=0.5),
        PoisonConfig(PoisonType.IMU_BIAS, start_time=7.0, duration=2.0, intensity=0.7, bias_value=1.0),
    ],
    'multi_sensor': [
        PoisonConfig(PoisonType.GPS_JUMP, start_time=2.0, duration=0.5, intensity=1.0, jump_distance=60.0),
        PoisonConfig(PoisonType.IMU_NOISE, start_time=4.0, duration=2.0, intensity=0.8, noise_stddev=0.3),
        PoisonConfig(PoisonType.ODOM_SCALING, start_time=8.0, duration=3.0, intensity=1.0, scale_factor=2.5),
    ],
    'intermittent': [
        PoisonConfig(PoisonType.GPS_JUMP, start_time=1.0, duration=0.3, intensity=1.0, jump_distance=40.0),
        PoisonConfig(PoisonType.GPS_JUMP, start_time=3.0, duration=0.3, intensity=1.0, jump_distance=45.0),
        PoisonConfig(PoisonType.GPS_JUMP, start_time=6.0, duration=0.3, intensity=1.0, jump_distance=50.0),
        PoisonConfig(PoisonType.GPS_JUMP, start_time=9.0, duration=0.3, intensity=1.0, jump_distance=55.0),
    ]
}


def get_preset_attack(name: str) -> List[PoisonConfig]:
    """Get a preset attack scenario by name"""
    return PRESET_ATTACKS.get(name, [])


def list_preset_attacks() -> List[str]:
    """List available preset attack scenarios"""
    return list(PRESET_ATTACKS.keys())

