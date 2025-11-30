import React, { useState, useEffect, useRef } from 'react';
import { Activity, AlertTriangle, CheckCircle, Database, Upload, Play, Pause, BarChart3, Settings } from 'lucide-react';

const ROSPoisoningDashboard = () => {
  const [isProcessing, setIsProcessing] = useState(false);
  const [sensorData, setSensorData] = useState({
    gps: { status: 'healthy', trustScore: 1.0, lastReading: null },
    imu: { status: 'healthy', trustScore: 1.0, lastReading: null },
    odometry: { status: 'healthy', trustScore: 1.0, lastReading: null }
  });
  const [anomalies, setAnomalies] = useState([]);
  const [metrics, setMetrics] = useState({
    totalReadings: 0,
    anomaliesDetected: 0,
    detectionRate: 0,
    avgProcessingTime: 0,
    currentTimestamp: 0
  });
  const [trajectoryData, setTrajectoryData] = useState([]);
  const [selectedFile, setSelectedFile] = useState(null);
  const [config, setConfig] = useState({
    predictionWindow: 5,
    velocityThreshold: 5.0,
    positionJumpThreshold: 10.0
  });
  const canvasRef = useRef(null);
  const animationRef = useRef(null);

  // Simulate ROS bag parsing and data processing
  const processROSBag = async (file) => {
    setIsProcessing(true);
    setMetrics(prev => ({ ...prev, totalReadings: 0, anomaliesDetected: 0 }));
    setAnomalies([]);
    setTrajectoryData([]);

    // Simulate reading ROS bag data
    const simulateDataStream = async () => {
      const startTime = Date.now();
      let dataPoints = 0;
      let detectedAnomalies = 0;
      const trajectory = [];
      const anomalyList = [];

      for (let i = 0; i < 200; i++) {
        if (!isProcessing) break;

        const timestamp = i * 0.1;
        
        // Simulate sensor readings with occasional poisoning
        const isPoisoned = i > 80 && i < 120;
        const lat = 33.5779 + (i * 0.00001) + (isPoisoned ? 0.0005 : 0);
        const lon = -101.8552 + (i * 0.00001) + (isPoisoned ? 0.0005 : 0);
        const velocity = 2.0 + (Math.random() - 0.5) * 0.5;

        // Update trajectory
        trajectory.push({ lat, lon, timestamp, isPoisoned });

        // Anomaly detection simulation
        if (isPoisoned && Math.random() > 0.3) {
          detectedAnomalies++;
          const anomaly = {
            timestamp,
            severity: 0.6 + Math.random() * 0.3,
            reasons: ['GPS position jump detected', 'GPS-Odometry mismatch'],
            position: { lat, lon }
          };
          anomalyList.push(anomaly);

          // Update trust scores
          setSensorData(prev => ({
            ...prev,
            gps: {
              ...prev.gps,
              status: 'warning',
              trustScore: Math.max(0.3, prev.gps.trustScore - 0.1),
              lastReading: timestamp
            }
          }));
        } else {
          // Reward good behavior
          setSensorData(prev => ({
            ...prev,
            gps: {
              ...prev.gps,
              status: prev.gps.trustScore > 0.8 ? 'healthy' : 'warning',
              trustScore: Math.min(1.0, prev.gps.trustScore + 0.01),
              lastReading: timestamp
            }
          }));
        }

        dataPoints++;

        // Update metrics
        const elapsed = (Date.now() - startTime) / 1000;
        setMetrics({
          totalReadings: dataPoints,
          anomaliesDetected: detectedAnomalies,
          detectionRate: dataPoints > 0 ? (detectedAnomalies / dataPoints * 100) : 0,
          avgProcessingTime: elapsed / dataPoints * 1000,
          currentTimestamp: timestamp
        });

        setTrajectoryData([...trajectory]);
        setAnomalies([...anomalyList]);

        await new Promise(resolve => setTimeout(resolve, 50));
      }

      setIsProcessing(false);
    };

    simulateDataStream();
  };

  // Draw trajectory on canvas
  useEffect(() => {
    if (!canvasRef.current || trajectoryData.length === 0) return;

    const canvas = canvasRef.current;
    const ctx = canvas.getContext('2d');
    const width = canvas.width;
    const height = canvas.height;

    // Clear canvas
    ctx.fillStyle = '#1e293b';
    ctx.fillRect(0, 0, width, height);

    // Draw grid
    ctx.strokeStyle = '#334155';
    ctx.lineWidth = 1;
    for (let i = 0; i < 10; i++) {
      ctx.beginPath();
      ctx.moveTo(i * width / 10, 0);
      ctx.lineTo(i * width / 10, height);
      ctx.stroke();
      ctx.beginPath();
      ctx.moveTo(0, i * height / 10);
      ctx.lineTo(width, i * height / 10);
      ctx.stroke();
    }

    if (trajectoryData.length < 2) return;

    // Calculate bounds
    const lats = trajectoryData.map(d => d.lat);
    const lons = trajectoryData.map(d => d.lon);
    const minLat = Math.min(...lats);
    const maxLat = Math.max(...lats);
    const minLon = Math.min(...lons);
    const maxLon = Math.max(...lons);

    const padding = 50;
    const scaleX = (width - 2 * padding) / (maxLon - minLon || 1);
    const scaleY = (height - 2 * padding) / (maxLat - minLat || 1);

    const toX = (lon) => padding + (lon - minLon) * scaleX;
    const toY = (lat) => height - (padding + (lat - minLat) * scaleY);

    // Draw trajectory
    ctx.strokeStyle = '#3b82f6';
    ctx.lineWidth = 2;
    ctx.beginPath();
    ctx.moveTo(toX(trajectoryData[0].lon), toY(trajectoryData[0].lat));

    for (let i = 1; i < trajectoryData.length; i++) {
      const point = trajectoryData[i];
      ctx.lineTo(toX(point.lon), toY(point.lat));
      
      // Change color for poisoned data
      if (point.isPoisoned && !trajectoryData[i-1].isPoisoned) {
        ctx.stroke();
        ctx.strokeStyle = '#ef4444';
        ctx.beginPath();
        ctx.moveTo(toX(point.lon), toY(point.lat));
      } else if (!point.isPoisoned && trajectoryData[i-1].isPoisoned) {
        ctx.stroke();
        ctx.strokeStyle = '#3b82f6';
        ctx.beginPath();
        ctx.moveTo(toX(point.lon), toY(point.lat));
      }
    }
    ctx.stroke();

    // Draw anomaly markers
    anomalies.forEach(anomaly => {
      ctx.fillStyle = '#ef4444';
      ctx.beginPath();
      ctx.arc(toX(anomaly.position.lon), toY(anomaly.position.lat), 6, 0, 2 * Math.PI);
      ctx.fill();
      
      ctx.strokeStyle = '#fef2f2';
      ctx.lineWidth = 2;
      ctx.stroke();
    });

    // Draw start point
    ctx.fillStyle = '#10b981';
    ctx.beginPath();
    ctx.arc(toX(trajectoryData[0].lon), toY(trajectoryData[0].lat), 8, 0, 2 * Math.PI);
    ctx.fill();

    // Draw current position
    const current = trajectoryData[trajectoryData.length - 1];
    ctx.fillStyle = '#8b5cf6';
    ctx.beginPath();
    ctx.arc(toX(current.lon), toY(current.lat), 8, 0, 2 * Math.PI);
    ctx.fill();

  }, [trajectoryData, anomalies]);

  const handleFileSelect = (event) => {
    const file = event.target.files[0];
    if (file) {
      setSelectedFile(file);
    }
  };

  const startProcessing = () => {
    if (selectedFile) {
      processROSBag(selectedFile);
    } else {
      // Start with synthetic data if no file selected
      processROSBag(null);
    }
  };

  const getSensorStatusColor = (status) => {
    switch (status) {
      case 'healthy': return 'text-green-500';
      case 'warning': return 'text-yellow-500';
      case 'critical': return 'text-red-500';
      default: return 'text-gray-500';
    }
  };

  const getTrustScoreColor = (score) => {
    if (score > 0.8) return 'bg-green-500';
    if (score > 0.5) return 'bg-yellow-500';
    return 'bg-red-500';
  };

  return (
    <div className="min-h-screen bg-slate-900 text-gray-100 p-6">
      {/* Header */}
      <div className="mb-6">
        <h1 className="text-3xl font-bold text-white mb-2">ROS Sensor Data Poisoning Detection</h1>
        <p className="text-gray-400">Real-time monitoring and anomaly detection for autonomous vehicle sensors</p>
      </div>

      {/* Control Panel */}
      <div className="bg-slate-800 rounded-lg p-6 mb-6 border border-slate-700">
        <div className="flex items-center gap-4 mb-4">
          <h2 className="text-xl font-semibold flex items-center gap-2">
            <Settings className="w-5 h-5" />
            Control Panel
          </h2>
        </div>
        
        <div className="grid grid-cols-1 md:grid-cols-3 gap-4 mb-4">
          <div>
            <label className="block text-sm text-gray-400 mb-2">ROS Bag File</label>
            <div className="flex gap-2">
              <input
                type="file"
                onChange={handleFileSelect}
                accept=".bag,.bag2"
                className="flex-1 bg-slate-700 border border-slate-600 rounded px-3 py-2 text-sm"
              />
            </div>
          </div>
          
          <div>
            <label className="block text-sm text-gray-400 mb-2">Prediction Window (s)</label>
            <input
              type="number"
              value={config.predictionWindow}
              onChange={(e) => setConfig({...config, predictionWindow: parseFloat(e.target.value)})}
              className="w-full bg-slate-700 border border-slate-600 rounded px-3 py-2"
              min="1"
              max="10"
              step="1"
            />
          </div>
          
          <div>
            <label className="block text-sm text-gray-400 mb-2">Max Velocity (m/s)</label>
            <input
              type="number"
              value={config.velocityThreshold}
              onChange={(e) => setConfig({...config, velocityThreshold: parseFloat(e.target.value)})}
              className="w-full bg-slate-700 border border-slate-600 rounded px-3 py-2"
              min="1"
              max="20"
              step="0.5"
            />
          </div>
        </div>

        <button
          onClick={startProcessing}
          disabled={isProcessing}
          className={`flex items-center gap-2 px-6 py-2 rounded font-medium ${
            isProcessing 
              ? 'bg-slate-600 cursor-not-allowed' 
              : 'bg-blue-600 hover:bg-blue-700'
          }`}
        >
          {isProcessing ? <Pause className="w-4 h-4" /> : <Play className="w-4 h-4" />}
          {isProcessing ? 'Processing...' : 'Start Analysis'}
        </button>
      </div>

      {/* Metrics Dashboard */}
      <div className="grid grid-cols-1 md:grid-cols-4 gap-4 mb-6">
        <div className="bg-slate-800 rounded-lg p-4 border border-slate-700">
          <div className="flex items-center justify-between mb-2">
            <span className="text-gray-400 text-sm">Total Readings</span>
            <Database className="w-5 h-5 text-blue-500" />
          </div>
          <div className="text-2xl font-bold">{metrics.totalReadings}</div>
        </div>

        <div className="bg-slate-800 rounded-lg p-4 border border-slate-700">
          <div className="flex items-center justify-between mb-2">
            <span className="text-gray-400 text-sm">Anomalies Detected</span>
            <AlertTriangle className="w-5 h-5 text-red-500" />
          </div>
          <div className="text-2xl font-bold text-red-500">{metrics.anomaliesDetected}</div>
        </div>

        <div className="bg-slate-800 rounded-lg p-4 border border-slate-700">
          <div className="flex items-center justify-between mb-2">
            <span className="text-gray-400 text-sm">Detection Rate</span>
            <BarChart3 className="w-5 h-5 text-yellow-500" />
          </div>
          <div className="text-2xl font-bold">{metrics.detectionRate.toFixed(1)}%</div>
        </div>

        <div className="bg-slate-800 rounded-lg p-4 border border-slate-700">
          <div className="flex items-center justify-between mb-2">
            <span className="text-gray-400 text-sm">Avg Process Time</span>
            <Activity className="w-5 h-5 text-green-500" />
          </div>
          <div className="text-2xl font-bold">{metrics.avgProcessingTime.toFixed(2)}ms</div>
        </div>
      </div>

      {/* Main Content Grid */}
      <div className="grid grid-cols-1 lg:grid-cols-3 gap-6">
        {/* Trajectory Visualization */}
        <div className="lg:col-span-2 bg-slate-800 rounded-lg p-6 border border-slate-700">
          <h3 className="text-lg font-semibold mb-4 flex items-center gap-2">
            <Activity className="w-5 h-5" />
            GPS Trajectory Monitor
          </h3>
          <canvas
            ref={canvasRef}
            width={800}
            height={500}
            className="w-full border border-slate-600 rounded"
          />
          <div className="mt-4 flex gap-4 text-sm">
            <div className="flex items-center gap-2">
              <div className="w-4 h-4 bg-green-500 rounded-full"></div>
              <span>Start Point</span>
            </div>
            <div className="flex items-center gap-2">
              <div className="w-4 h-4 bg-blue-500 rounded-full"></div>
              <span>Normal Path</span>
            </div>
            <div className="flex items-center gap-2">
              <div className="w-4 h-4 bg-red-500 rounded-full"></div>
              <span>Poisoned Data / Anomaly</span>
            </div>
            <div className="flex items-center gap-2">
              <div className="w-4 h-4 bg-purple-500 rounded-full"></div>
              <span>Current Position</span>
            </div>
          </div>
        </div>

        {/* Sensor Trust Scores */}
        <div className="bg-slate-800 rounded-lg p-6 border border-slate-700">
          <h3 className="text-lg font-semibold mb-4">Sensor Trust Scores</h3>
          
          {Object.entries(sensorData).map(([sensor, data]) => (
            <div key={sensor} className="mb-6">
              <div className="flex items-center justify-between mb-2">
                <span className="text-sm font-medium uppercase">{sensor}</span>
                <span className={`text-sm font-semibold ${getSensorStatusColor(data.status)}`}>
                  {data.status}
                </span>
              </div>
              
              <div className="bg-slate-700 rounded-full h-3 overflow-hidden mb-2">
                <div
                  className={`h-full ${getTrustScoreColor(data.trustScore)} transition-all duration-300`}
                  style={{ width: `${data.trustScore * 100}%` }}
                />
              </div>
              
              <div className="flex justify-between text-xs text-gray-400">
                <span>Trust: {(data.trustScore * 100).toFixed(1)}%</span>
                {data.lastReading !== null && (
                  <span>Last: {data.lastReading.toFixed(2)}s</span>
                )}
              </div>
            </div>
          ))}

          <div className="mt-6 p-4 bg-slate-700 rounded border border-slate-600">
            <h4 className="text-sm font-semibold mb-2 flex items-center gap-2">
              <CheckCircle className="w-4 h-4 text-green-500" />
              System Status
            </h4>
            <div className="text-xs text-gray-300 space-y-1">
              <div>Current Time: {metrics.currentTimestamp.toFixed(2)}s</div>
              <div>Processing: {isProcessing ? 'Active' : 'Idle'}</div>
              <div>Window: {config.predictionWindow}s</div>
            </div>
          </div>
        </div>
      </div>

      {/* Anomaly Log */}
      <div className="mt-6 bg-slate-800 rounded-lg p-6 border border-slate-700">
        <h3 className="text-lg font-semibold mb-4 flex items-center gap-2">
          <AlertTriangle className="w-5 h-5 text-red-500" />
          Anomaly Detection Log
        </h3>
        
        <div className="overflow-y-auto max-h-64">
          {anomalies.length === 0 ? (
            <div className="text-center text-gray-500 py-8">
              No anomalies detected yet. Start processing to begin analysis.
            </div>
          ) : (
            <div className="space-y-2">
              {anomalies.slice().reverse().map((anomaly, idx) => (
                <div
                  key={idx}
                  className="bg-slate-700 border border-slate-600 rounded p-3 hover:bg-slate-650"
                >
                  <div className="flex items-start justify-between mb-2">
                    <div className="flex items-center gap-2">
                      <AlertTriangle className="w-4 h-4 text-red-500" />
                      <span className="font-medium text-sm">
                        Timestamp: {anomaly.timestamp.toFixed(2)}s
                      </span>
                    </div>
                    <span className={`text-xs font-semibold px-2 py-1 rounded ${
                      anomaly.severity > 0.7 ? 'bg-red-900 text-red-200' :
                      anomaly.severity > 0.4 ? 'bg-yellow-900 text-yellow-200' :
                      'bg-blue-900 text-blue-200'
                    }`}>
                      Severity: {(anomaly.severity * 100).toFixed(0)}%
                    </span>
                  </div>
                  
                  <div className="text-xs text-gray-300 space-y-1">
                    <div>Position: ({anomaly.position.lat.toFixed(6)}, {anomaly.position.lon.toFixed(6)})</div>
                    <div className="flex flex-wrap gap-1 mt-2">
                      {anomaly.reasons.map((reason, i) => (
                        <span key={i} className="bg-slate-600 px-2 py-1 rounded text-xs">
                          {reason}
                        </span>
                      ))}
                    </div>
                  </div>
                </div>
              ))}
            </div>
          )}
        </div>
      </div>

      {/* Footer Info */}
      <div className="mt-6 text-center text-sm text-gray-500">
        <p>Texas Tech University - Computer Science Department</p>
        <p>ROS-Based Husky UGV Sensor Data Poisoning Detection System</p>
      </div>
    </div>
  );
};

export default ROSPoisoningDashboard;