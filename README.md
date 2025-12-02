# ROS Sensor Data Poisoning Detection Dashboard

A web-based dashboard for detecting and validating sensor data poisoning attacks on ROS-based autonomous vehicles.

## Quick Start

```bash
# Install dependencies
pip install -r requirements.txt

# Run the server
python3 app.py

# Open in browser
http://localhost:5000
```

## Features

### 1. Live Dashboard
- Real-time processing metrics
- GPS trajectory visualization with poisoned segments highlighted
- Sensor trust score indicators
- Anomaly detection log

### 2. Poison Injection
Inject controlled poisoning attacks during bag file processing:
- **GPS Jump** - Sudden position jump
- **GPS Drift** - Gradual position drift
- **GPS Freeze** - Frozen GPS readings
- **IMU Noise** - Add noise to IMU
- **IMU Bias** - Add constant bias
- **Odometry Scaling** - Scale odometry values

### 3. Preset Attack Scenarios
- `quick_test` - Two GPS jumps at 2s and 5s
- `sustained_drift` - 5 m/s GPS drift for 5 seconds
- `sensor_freeze` - GPS freeze for 3 seconds
- `imu_attack` - IMU noise + bias attacks
- `multi_sensor` - Combined GPS, IMU, and Odometry

### 4. Bag File Support
- **ROS1**: `.bag` files
- **ROS2**: `.zip` files containing `.db3` and `metadata.yaml`

**Note:** All bag files are assumed to be clean/unpoisoned. Poisoning is injected during processing to simulate attacks.

---

## 🚧 Missing Features (To Be Implemented)

### Live Rover Sensor Readings
Currently missing real-time display of actual sensor values:

- **IMU Data Display**
  - Roll, Pitch, Yaw angles
  - Angular velocity (X, Y, Z)
  - Linear acceleration (X, Y, Z)
  
- **Odometry Data Display**
  - Position (X, Y, Z)
  - Linear velocity
  - Angular velocity
  
- **GPS Data Display**
  - Latitude, Longitude, Altitude
  - Fix quality/status

### Historical Data Charts
- Time-series graphs for sensor readings
- Orientation charts (roll, pitch, yaw over time)
- Velocity/acceleration charts

---

## 🔧 Areas for Improvement

### 1. Poison Injection Script
**Current limitations:**
- Poison injection happens in-memory during processing
- No visual indication of exactly when/where poison was injected
- Limited feedback on injection success

**Needed improvements:**
- Visual markers showing exact injection timestamps on trajectory
- Real-time injection status overlay
- Injection timeline visualization
- Ability to see "before vs after" poisoning comparison

### 2. Validation Testing
**Current limitations:**
- Basic detection rate, precision, recall metrics
- Requires manual processor ID input
- Limited historical validation tracking

**Needed improvements:**
- Automatic processor ID detection
- Side-by-side comparison of injected vs detected anomalies
- Detailed per-injection validation results
- Export validation reports

### 3. Data Visualization
**Current limitations:**
- Trust scores shown as simple progress bars
- No time-series charts for sensor data
- Static trajectory view

**Needed improvements:**
- **Bar charts** for sensor comparison
- **Line graphs** for trust score over time
- **Real-time updating charts** for IMU/Odometry/GPS
- Interactive trajectory zoom/pan
- Heatmap for anomaly density

### 4. Poison Injection Visibility
**Main Goal:** When processing clean bag files with injected poison, clearly show:
- Where poison was injected (timestamp markers)
- How the data was manipulated (original vs poisoned values)
- Visual diff between clean and poisoned trajectory
- Timeline showing injection windows

---

## Project Structure

```
husky-dashboard/
├── app.py                    # Flask server
├── backend.py                # ROS bag parsing & anomaly detection
├── poison_injector.py        # Poison injection system
├── advanced_dashboard.html   # Main dashboard UI
├── requirements.txt          # Python dependencies
├── uploads/                  # Uploaded bag files
└── README.md                 # This file
```

## API Endpoints

| Endpoint | Method | Description |
|----------|--------|-------------|
| `/api/health` | GET | Health check |
| `/api/upload` | POST | Upload bag file |
| `/api/process/start` | POST | Start processing |
| `/api/process/<id>/status` | GET | Get status |
| `/api/process/<id>/stop` | POST | Stop processing |
| `/api/poison/presets` | GET | List attack presets |
| `/api/poison/inject` | POST | Inject poison |
| `/api/poison/clear` | POST | Clear poisons |
| `/api/poison/status` | GET | Poison status |
| `/api/poison/validate` | POST | Validate detection |

## Usage

### Test with Synthetic Data
1. Open dashboard at `http://localhost:5000`
2. Click **"🧪 Synthetic"** to run quick test
3. Watch trajectory and anomalies appear

### Process Bag File with Poison Injection
1. Go to **"💉 Inject Poison"** tab
2. Select a preset (e.g., `quick_test`) or configure custom
3. Click **"Apply Preset"**
4. Go back to **"📊 Live Dashboard"** tab
5. Upload your `.bag` or `.zip` file
6. Click **"▶️ Start"**
7. Watch the poisoned trajectory - red segments show where poison was active

### Validate Detection
1. After processing completes
2. Go to **"✓ Validate"** tab
3. Click **"Run Validation"**
4. Enter processor ID (e.g., `processor_0`)
5. Review detection accuracy metrics

## Requirements

- Python 3.8+
- Flask
- Flask-CORS
- NumPy
- Pandas
- rosbags

## License

MIT
