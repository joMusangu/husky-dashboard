#!/usr/bin/env python3
"""
GPS Prediction Model from IMU and Odometer Data
Predicts future GPS coordinates using only IMU and Odometer readings
"""

import pandas as pd
import numpy as np
from typing import Dict, List, Tuple, Optional, Union
from pathlib import Path
import os
import warnings
warnings.filterwarnings('ignore')

# Deep learning imports
try:
    import torch
    import torch.nn as nn
    import torch.optim as optim
    from torch.utils.data import Dataset, DataLoader
    TORCH_AVAILABLE = True
except ImportError:
    TORCH_AVAILABLE = False
    print("Warning: PyTorch not available. Install with: pip install torch")

# Scikit-learn for fallback
from sklearn.preprocessing import StandardScaler, MinMaxScaler
from sklearn.metrics import mean_squared_error, mean_absolute_error
import joblib

# Geodesic distance calculation
from math import radians, cos, sin, asin, sqrt


# PyTorch-dependent classes - only define if PyTorch is available
if TORCH_AVAILABLE:
    class GPSPredictionDataset(Dataset):
        """PyTorch Dataset for GPS prediction"""
        
        def __init__(self, X: np.ndarray, y: np.ndarray):
            self.X = torch.FloatTensor(X)
            self.y = torch.FloatTensor(y)
        
        def __len__(self):
            return len(self.X)
        
        def __getitem__(self, idx):
            return self.X[idx], self.y[idx]


    class LSTMModel(nn.Module):
        """LSTM model for GPS prediction"""
        
        def __init__(self, input_size: int, hidden_size: int = 128, num_layers: int = 2, 
                     dropout: float = 0.2, output_size: int = 2):
            super(LSTMModel, self).__init__()
            self.hidden_size = hidden_size
            self.num_layers = num_layers
            
            self.lstm = nn.LSTM(input_size, hidden_size, num_layers, 
                               batch_first=True, dropout=dropout if num_layers > 1 else 0)
            self.fc = nn.Linear(hidden_size, output_size)
            self.dropout = nn.Dropout(dropout)
        
        def forward(self, x):
            # x shape: (batch, seq_len, features)
            lstm_out, _ = self.lstm(x)
            # Take the last output
            last_output = lstm_out[:, -1, :]
            output = self.fc(self.dropout(last_output))
            return output


    class GRUModel(nn.Module):
        """GRU model for GPS prediction"""
        
        def __init__(self, input_size: int, hidden_size: int = 128, num_layers: int = 2,
                     dropout: float = 0.2, output_size: int = 2):
            super(GRUModel, self).__init__()
            self.hidden_size = hidden_size
            self.num_layers = num_layers
            
            self.gru = nn.GRU(input_size, hidden_size, num_layers,
                             batch_first=True, dropout=dropout if num_layers > 1 else 0)
            self.fc = nn.Linear(hidden_size, output_size)
            self.dropout = nn.Dropout(dropout)
        
        def forward(self, x):
            gru_out, _ = self.gru(x)
            last_output = gru_out[:, -1, :]
            output = self.fc(self.dropout(last_output))
            return output


    class TransformerModel(nn.Module):
        """Transformer encoder model for GPS prediction"""
        
        def __init__(self, input_size: int, d_model: int = 128, nhead: int = 8,
                     num_layers: int = 2, dropout: float = 0.2, output_size: int = 2):
            super(TransformerModel, self).__init__()
            self.d_model = d_model
            
            # Project input to model dimension
            self.input_projection = nn.Linear(input_size, d_model)
            
            # Positional encoding
            self.pos_encoder = nn.Parameter(torch.randn(1, 1000, d_model))
            
            # Transformer encoder
            encoder_layer = nn.TransformerEncoderLayer(
                d_model=d_model, nhead=nhead, dropout=dropout, batch_first=True
            )
            self.transformer = nn.TransformerEncoder(encoder_layer, num_layers=num_layers)
            
            # Output projection
            self.fc = nn.Linear(d_model, output_size)
            self.dropout = nn.Dropout(dropout)
        
        def forward(self, x):
            # x shape: (batch, seq_len, features)
            seq_len = x.size(1)
            x = self.input_projection(x)
            x = x + self.pos_encoder[:, :seq_len, :]
            x = self.transformer(x)
            # Take the last output
            last_output = x[:, -1, :]
            output = self.fc(self.dropout(last_output))
            return output
else:
    # Placeholder classes when PyTorch is not available
    GPSPredictionDataset = None
    LSTMModel = None
    GRUModel = None
    TransformerModel = None


def haversine_distance(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """
    Calculate the great circle distance between two points on Earth (in meters)
    using the Haversine formula
    
    Args:
        lat1, lon1: Latitude and longitude of first point (in degrees)
        lat2, lon2: Latitude and longitude of second point (in degrees)
    
    Returns:
        Distance in meters
    """
    # Convert to radians
    lat1, lon1, lat2, lon2 = map(radians, [lat1, lon1, lat2, lon2])
    
    # Haversine formula
    dlat = lat2 - lat1
    dlon = lon2 - lon1
    a = sin(dlat/2)**2 + cos(lat1) * cos(lat2) * sin(dlon/2)**2
    c = 2 * asin(sqrt(a))
    
    # Earth radius in meters
    R = 6371000
    
    return R * c


def latlon_to_meters(lat: float, lon: float, ref_lat: float, ref_lon: float) -> Tuple[float, float]:
    """
    Convert latitude/longitude to local meters relative to a reference point
    
    Args:
        lat, lon: Target coordinates (degrees)
        ref_lat, ref_lon: Reference coordinates (degrees)
    
    Returns:
        (x_meters, y_meters) relative to reference
    """
    # Approximate conversion (good for small distances)
    lat_diff = lat - ref_lat
    lon_diff = lon - ref_lon
    
    # Meters per degree (approximate)
    lat_to_m = 111320.0  # meters per degree latitude
    lon_to_m = 111320.0 * cos(radians(ref_lat))  # meters per degree longitude
    
    x_meters = lon_diff * lon_to_m
    y_meters = lat_diff * lat_to_m
    
    return x_meters, y_meters


def meters_to_latlon(x_meters: float, y_meters: float, ref_lat: float, ref_lon: float) -> Tuple[float, float]:
    """
    Convert local meters to latitude/longitude relative to a reference point
    
    Args:
        x_meters, y_meters: Local coordinates in meters
        ref_lat, ref_lon: Reference coordinates (degrees)
    
    Returns:
        (latitude, longitude) in degrees
    """
    lat_to_m = 111320.0
    lon_to_m = 111320.0 * cos(radians(ref_lat))
    
    lon_diff = x_meters / lon_to_m
    lat_diff = y_meters / lat_to_m
    
    lat = ref_lat + lat_diff
    lon = ref_lon + lon_diff
    
    return lat, lon


class GPSPredictionModel:
    """
    Machine learning model to predict GPS coordinates from IMU and Odometer data
    """
    
    def __init__(self, 
                 prediction_interval: float = 5.0,  # seconds
                 model_type: str = 'lstm',  # 'lstm', 'gru', 'transformer'
                 hidden_size: int = 128,
                 num_layers: int = 2,
                 dropout: float = 0.2,
                 batch_size: int = 32,
                 learning_rate: float = 0.001,
                 num_epochs: int = 50,
                 model_dir: str = 'gps_models'):
        """
        Initialize GPS prediction model
        
        Args:
            prediction_interval: Time window in seconds (1-20, default 5)
            model_type: 'lstm', 'gru', or 'transformer'
            hidden_size: Hidden layer size for neural network
            num_layers: Number of layers
            dropout: Dropout rate
            batch_size: Training batch size
            learning_rate: Learning rate
            num_epochs: Number of training epochs
            model_dir: Directory to save models
        """
        if not TORCH_AVAILABLE:
            raise ImportError("PyTorch is required. Install with: pip install torch")
        
        if not (1.0 <= prediction_interval <= 20.0):
            raise ValueError("prediction_interval must be between 1 and 20 seconds")
        
        self.prediction_interval = prediction_interval
        self.model_type = model_type.lower()
        self.hidden_size = hidden_size
        self.num_layers = num_layers
        self.dropout = dropout
        self.batch_size = batch_size
        self.learning_rate = learning_rate
        self.num_epochs = num_epochs
        self.model_dir = model_dir
        
        # Create model directory
        os.makedirs(self.model_dir, exist_ok=True)
        
        # Model and scalers
        self.model = None
        self.input_scaler = StandardScaler()
        self.output_scaler = StandardScaler()
        self.ref_lat = None
        self.ref_lon = None
        
        # Training history
        self.train_losses = []
        self.val_losses = []
        
    def load_data(self, data_dir: str) -> Dict[str, pd.DataFrame]:
        """
        Load GPS, IMU, and Odometer data from directory
        
        Args:
            data_dir: Directory containing navsat-fix.csv, imu-data.csv, odometry-filtered.csv
        
        Returns:
            Dictionary with 'gps', 'imu', 'odom' DataFrames
        """
        data_dir = Path(data_dir)
        
        gps_file = data_dir / 'navsat-fix.csv'
        imu_file = data_dir / 'imu-data.csv'
        odom_file = data_dir / 'odometry-filtered.csv'
        
        if not gps_file.exists():
            raise FileNotFoundError(f"GPS file not found: {gps_file}")
        if not imu_file.exists():
            raise FileNotFoundError(f"IMU file not found: {imu_file}")
        if not odom_file.exists():
            raise FileNotFoundError(f"Odometer file not found: {odom_file}")
        
        # Load data
        gps_df = pd.read_csv(gps_file)
        imu_df = pd.read_csv(imu_file)
        odom_df = pd.read_csv(odom_file)
        
        # Convert Time to numeric and sort
        for df in [gps_df, imu_df, odom_df]:
            df['Time'] = pd.to_numeric(df['Time'], errors='coerce')
            df.dropna(subset=['Time'], inplace=True)
            df.sort_values('Time', inplace=True)
            df.reset_index(drop=True, inplace=True)
        
        return {
            'gps': gps_df,
            'imu': imu_df,
            'odom': odom_df
        }
    
    def extract_features(self, imu_df: pd.DataFrame, odom_df: pd.DataFrame, 
                        start_time: float, end_time: float) -> np.ndarray:
        """
        Extract features from IMU and Odometer data in time window
        
        Args:
            imu_df: IMU DataFrame
            odom_df: Odometer DataFrame
            start_time: Start time of window
            end_time: End time of window
        
        Returns:
            Feature array of shape (seq_len, n_features)
        """
        # Filter data in time window
        imu_window = imu_df[(imu_df['Time'] >= start_time) & (imu_df['Time'] <= end_time)].copy()
        odom_window = odom_df[(odom_df['Time'] >= start_time) & (odom_df['Time'] <= end_time)].copy()
        
        if len(imu_window) == 0 or len(odom_window) == 0:
            return None
        
        # Interpolate to common time grid (use IMU timestamps as base, higher frequency)
        time_grid = imu_window['Time'].values
        
        # Extract IMU features
        imu_features = []
        imu_cols = [
            'linear_acceleration.x', 'linear_acceleration.y', 'linear_acceleration.z',
            'angular_velocity.x', 'angular_velocity.y', 'angular_velocity.z'
        ]
        for col in imu_cols:
            if col in imu_window.columns:
                imu_features.append(imu_window[col].values)
            else:
                imu_features.append(np.zeros(len(imu_window)))
        
        # Extract Odometer features
        odom_features = []
        odom_cols = [
            'twist.twist.linear.x', 'twist.twist.linear.y', 'twist.twist.linear.z',
            'twist.twist.angular.x', 'twist.twist.angular.y', 'twist.twist.angular.z'
        ]
        for col in odom_cols:
            if col in odom_window.columns:
                # Interpolate to time grid
                values = np.interp(time_grid, odom_window['Time'].values, odom_window[col].values)
                odom_features.append(values)
            else:
                odom_features.append(np.zeros(len(time_grid)))
        
        # Combine features
        features = np.column_stack(imu_features + odom_features)
        
        return features
    
    def prepare_training_data(self, data_dict: Dict[str, pd.DataFrame], 
                             step_size: float = 0.1) -> Tuple[np.ndarray, np.ndarray]:
        """
        Prepare training data using sliding windows
        
        Args:
            data_dict: Dictionary with 'gps', 'imu', 'odom' DataFrames
            step_size: Step size for sliding window (seconds)
        
        Returns:
            (X, y) where X is (n_samples, seq_len, n_features) and y is (n_samples, 2) [lat, lon]
        """
        gps_df = data_dict['gps']
        imu_df = data_dict['imu']
        odom_df = data_dict['odom']
        
        # Set reference point (first GPS reading)
        if self.ref_lat is None:
            self.ref_lat = gps_df.iloc[0]['latitude']
            self.ref_lon = gps_df.iloc[0]['longitude']
        
        X_samples = []
        y_samples = []
        
        # Get time range
        min_time = max(gps_df['Time'].min(), imu_df['Time'].min(), odom_df['Time'].min())
        max_time = min(gps_df['Time'].max(), imu_df['Time'].max(), odom_df['Time'].max())
        
        # Sliding window
        current_time = min_time
        while current_time + self.prediction_interval <= max_time:
            # Get GPS at start and end
            gps_start = gps_df[gps_df['Time'] <= current_time]
            gps_end = gps_df[gps_df['Time'] >= current_time + self.prediction_interval]
            
            if len(gps_start) == 0 or len(gps_end) == 0:
                current_time += step_size
                continue
            
            gps_start_val = gps_start.iloc[-1]
            gps_end_val = gps_end.iloc[0]
            
            # Extract features from IMU and Odometer in the window
            features = self.extract_features(imu_df, odom_df, 
                                           current_time, 
                                           current_time + self.prediction_interval)
            
            if features is None or len(features) == 0:
                current_time += step_size
                continue
            
            # Target: GPS at end time (convert to meters relative to start)
            start_lat, start_lon = gps_start_val['latitude'], gps_start_val['longitude']
            end_lat, end_lon = gps_end_val['latitude'], gps_end_val['longitude']
            
            # Convert to meters
            x_meters, y_meters = latlon_to_meters(end_lat, end_lon, start_lat, start_lon)
            
            X_samples.append(features)
            y_samples.append([x_meters, y_meters])
            
            current_time += step_size
        
        if len(X_samples) == 0:
            raise ValueError("No valid training samples found. Check data synchronization.")
        
        # Pad sequences to same length (or truncate)
        max_len = max(len(x) for x in X_samples)
        min_len = min(len(x) for x in X_samples)
        
        # Use median length for efficiency
        target_len = int(np.median([len(x) for x in X_samples]))
        
        X_padded = []
        for x in X_samples:
            if len(x) > target_len:
                # Truncate
                x = x[:target_len]
            elif len(x) < target_len:
                # Pad with last value
                padding = np.tile(x[-1:], (target_len - len(x), 1))
                x = np.vstack([x, padding])
            X_padded.append(x)
        
        X = np.array(X_padded)
        y = np.array(y_samples)
        
        return X, y
    
    def train(self, data_dirs: List[str], validation_split: float = 0.2):
        """
        Train the model on multiple data directories
        
        Args:
            data_dirs: List of directories containing training data
            validation_split: Fraction of data to use for validation
        """
        print(f"Training GPS prediction model (interval={self.prediction_interval}s, type={self.model_type})")
        print(f"Loading data from {len(data_dirs)} directories...")
        
        # Load and combine all data
        all_X = []
        all_y = []
        
        for data_dir in data_dirs:
            try:
                data_dict = self.load_data(data_dir)
                X, y = self.prepare_training_data(data_dict)
                all_X.append(X)
                all_y.append(y)
                print(f"  Loaded {len(X)} samples from {Path(data_dir).name}")
            except Exception as e:
                print(f"  Warning: Failed to load {data_dir}: {e}")
                continue
        
        if len(all_X) == 0:
            raise ValueError("No valid training data found")
        
        # Combine all data
        X = np.vstack(all_X)
        y = np.vstack(all_y)
        
        print(f"Total samples: {len(X)}")
        print(f"Sequence length: {X.shape[1]}, Features: {X.shape[2]}")
        
        # Split train/validation
        n_train = int(len(X) * (1 - validation_split))
        indices = np.random.permutation(len(X))
        train_indices = indices[:n_train]
        val_indices = indices[n_train:]
        
        X_train, y_train = X[train_indices], y[train_indices]
        X_val, y_val = X[val_indices], y[val_indices]
        
        # Scale features
        X_train_flat = X_train.reshape(-1, X_train.shape[-1])
        X_val_flat = X_val.reshape(-1, X_val.shape[-1])
        
        self.input_scaler.fit(X_train_flat)
        X_train_scaled = self.input_scaler.transform(X_train_flat).reshape(X_train.shape)
        X_val_scaled = self.input_scaler.transform(X_val_flat).reshape(X_val.shape)
        
        # Scale outputs
        self.output_scaler.fit(y_train)
        y_train_scaled = self.output_scaler.transform(y_train)
        y_val_scaled = self.output_scaler.transform(y_val)
        
        # Create model
        input_size = X_train.shape[2]
        output_size = 2  # x, y in meters
        
        if self.model_type == 'lstm':
            self.model = LSTMModel(input_size, self.hidden_size, self.num_layers, 
                                  self.dropout, output_size)
        elif self.model_type == 'gru':
            self.model = GRUModel(input_size, self.hidden_size, self.num_layers,
                                 self.dropout, output_size)
        elif self.model_type == 'transformer':
            self.model = TransformerModel(input_size, self.hidden_size, 
                                         nhead=8, num_layers=self.num_layers,
                                         dropout=self.dropout, output_size=output_size)
        else:
            raise ValueError(f"Unknown model type: {self.model_type}")
        
        # Training setup
        device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
        self.model.to(device)
        
        criterion = nn.MSELoss()
        optimizer = optim.Adam(self.model.parameters(), lr=self.learning_rate)
        
        # Create datasets
        train_dataset = GPSPredictionDataset(X_train_scaled, y_train_scaled)
        val_dataset = GPSPredictionDataset(X_val_scaled, y_val_scaled)
        
        train_loader = DataLoader(train_dataset, batch_size=self.batch_size, shuffle=True)
        val_loader = DataLoader(val_dataset, batch_size=self.batch_size, shuffle=False)
        
        # Training loop
        print("\nTraining...")
        best_val_loss = float('inf')
        
        for epoch in range(self.num_epochs):
            # Train
            self.model.train()
            train_loss = 0.0
            for X_batch, y_batch in train_loader:
                X_batch, y_batch = X_batch.to(device), y_batch.to(device)
                
                optimizer.zero_grad()
                outputs = self.model(X_batch)
                loss = criterion(outputs, y_batch)
                loss.backward()
                optimizer.step()
                
                train_loss += loss.item()
            
            # Validate
            self.model.eval()
            val_loss = 0.0
            with torch.no_grad():
                for X_batch, y_batch in val_loader:
                    X_batch, y_batch = X_batch.to(device), y_batch.to(device)
                    outputs = self.model(X_batch)
                    loss = criterion(outputs, y_batch)
                    val_loss += loss.item()
            
            train_loss /= len(train_loader)
            val_loss /= len(val_loader)
            
            self.train_losses.append(train_loss)
            self.val_losses.append(val_loss)
            
            if (epoch + 1) % 10 == 0 or epoch == 0:
                print(f"Epoch {epoch+1}/{self.num_epochs} - Train Loss: {train_loss:.6f}, Val Loss: {val_loss:.6f}")
            
            # Save best model
            if val_loss < best_val_loss:
                best_val_loss = val_loss
                self.save_model()
        
        print(f"\nTraining complete! Best validation loss: {best_val_loss:.6f}")
        
        # Load best model
        self.load_model()
    
    def predict(self, imu_df: pd.DataFrame, odom_df: pd.DataFrame,
               start_gps: Tuple[float, float], start_time: float) -> Tuple[float, float, float]:
        """
        Predict GPS coordinates at start_time + prediction_interval
        
        Args:
            imu_df: IMU DataFrame
            odom_df: Odometer DataFrame
            start_gps: (latitude, longitude) at start_time
            start_time: Start time
        
        Returns:
            (predicted_lat, predicted_lon, error_meters) where error is distance from start
        """
        if self.model is None:
            raise ValueError("Model not trained. Call train() first.")
        
        end_time = start_time + self.prediction_interval
        
        # Extract features
        features = self.extract_features(imu_df, odom_df, start_time, end_time)
        
        if features is None or len(features) == 0:
            raise ValueError("No features extracted for prediction window")
        
        # Scale features
        features_flat = features.reshape(-1, features.shape[-1])
        features_scaled = self.input_scaler.transform(features_flat)
        features_scaled = features_scaled.reshape(1, -1, features.shape[-1])
        
        # Predict
        device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
        self.model.eval()
        with torch.no_grad():
            X_tensor = torch.FloatTensor(features_scaled).to(device)
            y_pred_scaled = self.model(X_tensor).cpu().numpy()[0]
        
        # Unscale
        y_pred = self.output_scaler.inverse_transform([y_pred_scaled])[0]
        
        # Convert meters to lat/lon
        start_lat, start_lon = start_gps
        x_meters, y_meters = y_pred[0], y_pred[1]
        pred_lat, pred_lon = meters_to_latlon(x_meters, y_meters, start_lat, start_lon)
        
        # Calculate error (distance from start)
        error = haversine_distance(start_lat, start_lon, pred_lat, pred_lon)
        
        return pred_lat, pred_lon, error
    
    def evaluate(self, data_dir: str, error_threshold: float = 50.0) -> Dict:
        """
        Evaluate model on test data
        
        Args:
            data_dir: Directory containing test data
            error_threshold: Distance threshold in meters for classification
        
        Returns:
            Dictionary with evaluation metrics
        """
        data_dict = self.load_data(data_dir)
        gps_df = data_dict['gps']
        imu_df = data_dict['imu']
        odom_df = data_dict['odom']
        
        predictions = []
        actuals = []
        errors = []
        classifications = []
        
        # Get time range
        min_time = max(gps_df['Time'].min(), imu_df['Time'].min(), odom_df['Time'].min())
        max_time = min(gps_df['Time'].max(), imu_df['Time'].max(), odom_df['Time'].max())
        
        step_size = 1.0  # Evaluate every second
        current_time = min_time
        
        while current_time + self.prediction_interval <= max_time:
            # Get GPS at start and end
            gps_start = gps_df[gps_df['Time'] <= current_time]
            gps_end = gps_df[gps_df['Time'] >= current_time + self.prediction_interval]
            
            if len(gps_start) == 0 or len(gps_end) == 0:
                current_time += step_size
                continue
            
            start_gps = (gps_start.iloc[-1]['latitude'], gps_start.iloc[-1]['longitude'])
            end_gps = (gps_end.iloc[0]['latitude'], gps_end.iloc[0]['longitude'])
            
            try:
                # Predict
                pred_lat, pred_lon, pred_error = self.predict(imu_df, odom_df, start_gps, current_time)
                
                # Actual error (distance from predicted to actual)
                actual_error = haversine_distance(pred_lat, pred_lon, end_gps[0], end_gps[1])
                
                # Classification
                is_valid = actual_error <= error_threshold
                
                predictions.append((pred_lat, pred_lon))
                actuals.append(end_gps)
                errors.append(actual_error)
                classifications.append(is_valid)
                
            except Exception as e:
                print(f"Warning: Prediction failed at t={current_time}: {e}")
            
            current_time += step_size
        
        if len(errors) == 0:
            raise ValueError("No valid predictions made")
        
        errors = np.array(errors)
        classifications = np.array(classifications)
        
        results = {
            'n_samples': len(errors),
            'mean_error': float(np.mean(errors)),
            'median_error': float(np.median(errors)),
            'std_error': float(np.std(errors)),
            'rmse': float(np.sqrt(np.mean(errors**2))),
            'max_error': float(np.max(errors)),
            'min_error': float(np.min(errors)),
            'threshold': error_threshold,
            'valid_count': int(np.sum(classifications)),
            'falsified_count': int(np.sum(~classifications)),
            'valid_rate': float(np.mean(classifications)),
            'predictions': predictions,
            'actuals': actuals,
            'errors': errors.tolist(),
            'classifications': classifications.tolist()
        }
        
        return results
    
    def save_model(self):
        """Save model and scalers"""
        model_path = os.path.join(self.model_dir, f'gps_model_{self.model_type}_{self.prediction_interval}s.pt')
        scaler_input_path = os.path.join(self.model_dir, f'input_scaler_{self.prediction_interval}s.pkl')
        scaler_output_path = os.path.join(self.model_dir, f'output_scaler_{self.prediction_interval}s.pkl')
        metadata_path = os.path.join(self.model_dir, f'metadata_{self.prediction_interval}s.pkl')
        
        torch.save(self.model.state_dict(), model_path)
        joblib.dump(self.input_scaler, scaler_input_path)
        joblib.dump(self.output_scaler, scaler_output_path)
        # Get input size from model or scaler
        input_size = self.input_scaler.n_features_in_ if hasattr(self.input_scaler, 'n_features_in_') else 12
        
        joblib.dump({
            'ref_lat': self.ref_lat,
            'ref_lon': self.ref_lon,
            'model_type': self.model_type,
            'prediction_interval': self.prediction_interval,
            'hidden_size': self.hidden_size,
            'num_layers': self.num_layers,
            'input_size': input_size
        }, metadata_path)
        
        print(f"Model saved to {model_path}")
    
    def load_model(self):
        """Load model and scalers"""
        model_path = os.path.join(self.model_dir, f'gps_model_{self.model_type}_{self.prediction_interval}s.pt')
        scaler_input_path = os.path.join(self.model_dir, f'input_scaler_{self.prediction_interval}s.pkl')
        scaler_output_path = os.path.join(self.model_dir, f'output_scaler_{self.prediction_interval}s.pkl')
        metadata_path = os.path.join(self.model_dir, f'metadata_{self.prediction_interval}s.pkl')
        
        if not os.path.exists(model_path):
            raise FileNotFoundError(f"Model not found: {model_path}")
        
        # Load metadata
        metadata = joblib.load(metadata_path)
        self.ref_lat = metadata['ref_lat']
        self.ref_lon = metadata['ref_lon']
        
        # Load scalers
        self.input_scaler = joblib.load(scaler_input_path)
        self.output_scaler = joblib.load(scaler_output_path)
        
        # Get input size from metadata or scaler
        if 'input_size' in metadata:
            input_size = metadata['input_size']
        elif hasattr(self.input_scaler, 'n_features_in_'):
            input_size = self.input_scaler.n_features_in_
        else:
            # Fallback: assume 12 features (6 IMU + 6 Odometer)
            input_size = 12
        
        if self.model_type == 'lstm':
            self.model = LSTMModel(input_size, self.hidden_size, self.num_layers,
                                  self.dropout, 2)
        elif self.model_type == 'gru':
            self.model = GRUModel(input_size, self.hidden_size, self.num_layers,
                                 self.dropout, 2)
        elif self.model_type == 'transformer':
            self.model = TransformerModel(input_size, self.hidden_size,
                                        nhead=8, num_layers=self.num_layers,
                                        dropout=self.dropout, output_size=2)
        
        self.model.load_state_dict(torch.load(model_path, map_location='cpu'))
        self.model.eval()
        
        print(f"Model loaded from {model_path}")

