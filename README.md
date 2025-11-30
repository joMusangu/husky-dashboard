# ROS Sensor Data Poisoning Detection System

A real-time dashboard for detecting sensor data poisoning in ROS-based autonomous vehicle systems, specifically designed for Husky UGV platforms.

## Features

- **Real-time Anomaly Detection**: Detects GPS position jumps, velocity inconsistencies, and cross-sensor mismatches
- **ROS Bag File Support**: Parse and analyze ROS1 and ROS2 bag files
- **CAN Bus Integration**: Support for CAN bus log parsing (optional)
- **Interactive Dashboard**: Real-time visualization of sensor trajectories, trust scores, and anomaly logs
- **Synthetic Data Mode**: Test the system without ROS bag files using generated data

## Project Structure

```
CAPSTONE PROJECT/
├── backend.py          # Core detection algorithms and ROS bag parsing
├── app.py              # Flask API server (connects frontend to backend)
├── frontend.js         # React component (original, for reference)
├── index.html          # HTML file with integrated React frontend
├── requirements.txt    # Python dependencies
└── README.md          # This file
```

## Prerequisites

- Python 3.8 or higher
- pip (Python package manager)

### Optional Dependencies (for ROS bag file support)

If you want to process actual ROS bag files, install:

```bash
# For ROS1 bag files
pip install rosbag

# For ROS2 bag files
pip install rosbags

# For CAN bus log files
pip install python-can
```

## Installation

1. **Clone or navigate to the project directory**:
   ```bash
   cd "C:\Users\mjtsh\Desktop\CAPSTONE PROJECT"
   ```

2. **Create a virtual environment (recommended)**:
   ```bash
   python -m venv venv
   ```

3. **Activate the virtual environment**:
   - On Windows (PowerShell):
     ```powershell
     .\venv\Scripts\Activate.ps1
     ```
   - On Windows (Command Prompt):
     ```cmd
     venv\Scripts\activate.bat
     ```
   - On Linux/Mac:
     ```bash
     source venv/bin/activate
     ```

4. **Install Python dependencies**:
   ```bash
   pip install -r requirements.txt
   ```

## Running the Application

### Step 1: Start the Flask Backend Server

Open a terminal/command prompt and run:

```bash
python app.py
```

You should see output like:
```
======================================================================
ROS Sensor Data Poisoning Detection - Flask API Server
======================================================================

Starting server on http://localhost:5000
Frontend will be available at http://localhost:5000

Press Ctrl+C to stop the server
======================================================================
```

### Step 2: Open the Frontend

Open your web browser and navigate to:

```
http://localhost:5000
```

The dashboard should load automatically.

## Usage

### Using Synthetic Data (Default)

1. Leave the "ROS Bag File" field empty
2. Adjust configuration parameters if needed:
   - **Prediction Window**: Number of seconds to use for prediction (default: 5)
   - **Max Velocity**: Maximum expected velocity in m/s (default: 5.0)
3. Click **"Start Analysis"**
4. Watch the real-time dashboard update with:
   - GPS trajectory visualization
   - Sensor trust scores
   - Anomaly detection log
   - Processing metrics

### Using ROS Bag Files

1. Click **"Choose File"** in the ROS Bag File field
2. Select a `.bag` file (ROS1 or ROS2 format)
3. Click **"Start Analysis"**
4. The system will parse the bag file and process the sensor data

### Stopping Processing

Click **"Stop Processing"** at any time to halt the analysis.

## API Endpoints

The Flask server provides the following API endpoints:

- `GET /` - Serves the frontend HTML page
- `GET /api/health` - Health check endpoint
- `POST /api/process/start` - Start processing (synthetic or ROS bag file)
- `GET /api/process/<processor_id>/status` - Get current processing status
- `POST /api/process/<processor_id>/stop` - Stop processing
- `POST /api/upload` - Upload ROS bag file

## How It Works

1. **Data Collection**: The system reads sensor data from ROS bag files or generates synthetic data
2. **Anomaly Detection**: Multiple detection algorithms run in parallel:
   - Temporal consistency checks (velocity, position jumps)
   - Cross-sensor validation (GPS vs. odometry)
   - Prediction-based detection
3. **Trust Scoring**: Each sensor gets a dynamic trust score based on detected anomalies
4. **Real-time Updates**: The frontend polls the backend every 500ms for status updates
5. **Visualization**: Trajectory, anomalies, and metrics are displayed in real-time

## Troubleshooting

### Port Already in Use

If port 5000 is already in use, edit `app.py` and change:
```python
app.run(debug=True, host='0.0.0.0', port=5000)
```
to a different port (e.g., `port=5001`), then access `http://localhost:5001`

### CORS Errors

If you see CORS errors in the browser console, make sure `flask-cors` is installed:
```bash
pip install flask-cors
```

### ROS Bag Parsing Errors

If you get errors parsing ROS bag files:
- Make sure you have the correct ROS bag parsing library installed (`rosbag` for ROS1, `rosbags` for ROS2)
- Check that your bag file is not corrupted
- The system will fall back to synthetic data if parsing fails

### Frontend Not Loading

- Make sure the Flask server is running
- Check the browser console for errors
- Try refreshing the page
- Make sure you're accessing `http://localhost:5000` (not `https://`)

## Development

### Running Backend Only

To test the backend independently:

```bash
python backend.py
```

This will run the example code in `backend.py` with synthetic data.

### Modifying the Frontend

The frontend is embedded in `index.html` using React via CDN. To modify it:
1. Edit the React component code in the `<script type="text/babel">` section
2. Refresh the browser to see changes

## License

This project is part of a capstone project at Texas Tech University - Computer Science Department.

## Support

For issues or questions, please refer to the project documentation or contact the development team.

