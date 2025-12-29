"""
U2U-MCS LSTM Agent
==================

Python agent for AI-driven dynamic MCS selection task.

Protocol Messages:
- hello -> helloAck (with schema validation)
- reset -> action (reset state, return initial action)
- step -> action (process observation, return action)
- close -> (graceful shutdown)

Error Codes:
- SCHEMA_MISMATCH: Schema hash doesn't match
- VALIDATION_ERROR: Invalid observation/action
- PROTOCOL_ERROR: Message format error
- INTERNAL_ERROR: Agent internal error

Usage:
    python u2u_mcs_lstm_agent.py
"""

import sys
import json
import time
import numpy as np
import traceback

# Try importing torch
try:
    import torch
    import torch.nn as nn
    import torch.optim as optim
    HAS_TORCH = True
except ImportError:
    HAS_TORCH = False

# ==================== Error Codes ====================

ERROR_CODES = {
    'SCHEMA_MISMATCH': 'SCHEMA_MISMATCH',
    'VALIDATION_ERROR': 'VALIDATION_ERROR',
    'PROTOCOL_ERROR': 'PROTOCOL_ERROR',
    'INTERNAL_ERROR': 'INTERNAL_ERROR'
}

# ==================== Logging ====================

def log(msg):
    """Log to stderr (protocol uses stdout)"""
    print(f"[Agent] {msg}", file=sys.stderr, flush=True)

# ==================== Protocol ====================

def read_message():
    """Read a JSON message from stdin"""
    try:
        line = sys.stdin.readline()
        if not line:
            return None
        return json.loads(line.strip())
    except json.JSONDecodeError as e:
        log(f"Invalid JSON: {e}")
        return None

def send_message(msg):
    """Send a JSON message to stdout"""
    print(json.dumps(msg), flush=True)

def create_response(req_msg, data, msg_type=None):
    """Create a response message"""
    return {
        "v": 1,
        "type": msg_type or req_msg["type"],
        "sessionId": req_msg.get("sessionId", ""),
        "seq": req_msg.get("seq", 0),
        "timestamp": int(time.time() * 1000),
        "requestId": req_msg.get("requestId"),
        "data": data
    }

def create_error_response(req_msg, code, message, details=None):
    """Create an error response"""
    data = {
        "code": code,
        "message": message
    }
    if details:
        data["details"] = details
    return create_response(req_msg, data, "error")

# ==================== LSTM Agent ====================

class LSTMAgent:
    """LSTM-based MCS prediction agent (uavlink port)"""
    
    def __init__(self, obs_dim=7, act_dim=23):
        self.obs_dim = obs_dim
        self.act_dim = act_dim
        
        # Online learning parameters
        self.input_len = 20
        self.pred_len = 5
        self.batch_size = 2
        self.alpha = 0.6
        self.delta = 1
        
        # Initialize buffers
        self._init_buffers()
        
        # Model
        if HAS_TORCH:
            self.model = LSTMModel(1, 20, 1)
            self.optimizer = optim.Adam(self.model.parameters(), lr=0.001)
            self.criterion = nn.MSELoss()
            log("LSTM model initialized")
        else:
            log("WARNING: PyTorch not found, using heuristic fallback")
    
    def _init_buffers(self):
        """Initialize/reset all buffers"""
        self.sinr_queue = np.empty((0, 1), dtype=np.float32)
        self.mcs_queue = np.empty((0, 1), dtype=np.float32)
        self.delay_queue = np.empty((0, 1), dtype=np.float32)
        self.delay_queue_mcs = np.empty((0, 1), dtype=np.float32)
        self.train_data = np.empty((0, self.input_len, 1), dtype=np.float32)
        self.target = np.empty((0, 1), dtype=np.float32)
        self.prediction = np.empty((0, 1), dtype=np.float32)
        self.last_mcs_hist = np.empty((0, 1), dtype=np.float32)
        self.corrected_predict = np.empty((0, 1), dtype=np.float32)
        self.step_count = 0
    
    def reset(self):
        """Reset agent state for new episode"""
        self._init_buffers()
        log("Agent reset")
        return self.select_action([0] * self.obs_dim, 0, False)
    
    def select_action(self, observation, reward, done):
        """Select MCS action based on observation"""
        try:
            # Denormalize SINR (observation[0] is normalized by /30)
            sinr = observation[0] * 30 if len(observation) > 0 else 0
            
            # Simple heuristic if no torch
            if not HAS_TORCH:
                return self._sinr_to_mcs(sinr)
            
            # Get current MCS target
            current_mcs = float(self._sinr_to_mcs(sinr))
            
            # Update delay queues
            self.delay_queue = np.append(self.delay_queue, [[sinr]], axis=0)
            self.delay_queue_mcs = np.append(self.delay_queue_mcs, [[current_mcs]], axis=0)
            
            # Get delayed values
            if len(self.delay_queue) < self.delta:
                delayed_sinr = self.delay_queue[-1]
                delayed_mcs = self.delay_queue_mcs[-1]
            else:
                delayed_sinr = self.delay_queue[-self.delta]
                delayed_mcs = self.delay_queue_mcs[-self.delta]
            
            # Update main queues
            self.sinr_queue = np.append(self.sinr_queue, [delayed_sinr], axis=0)
            self.mcs_queue = np.append(self.mcs_queue, [delayed_mcs], axis=0)
            
            predicted_mcs = current_mcs
            
            # Check if we have enough data
            if len(self.sinr_queue) >= self.input_len + self.delta:
                self.target = np.append(self.target, [delayed_mcs], axis=0)
            
            if len(self.sinr_queue) >= self.input_len:
                # Form input sequence
                one_data = self.sinr_queue[-self.input_len:]
                self.train_data = np.append(self.train_data, [one_data], axis=0)
                
                # Predict
                input_tensor = torch.FloatTensor(one_data).unsqueeze(0)
                with torch.no_grad():
                    pred_out = self.model(input_tensor).item()
                
                self.prediction = np.append(self.prediction, [[pred_out]], axis=0)
                self.last_mcs_hist = np.append(self.last_mcs_hist, [delayed_mcs], axis=0)
                self.corrected_predict = np.append(self.corrected_predict, [[pred_out]], axis=0)
                
                predicted_mcs = pred_out
                
                # Online training
                if len(self.train_data) >= self.pred_len + self.delta:
                    self._online_training()
                    predicted_mcs = self.corrected_predict[-1][0]
            
            self.step_count += 1
            return int(max(0, min(22, round(predicted_mcs))))
            
        except Exception as e:
            log(f"Error in select_action: {e}")
            return self._sinr_to_mcs(observation[0] * 30 if observation else 0)
    
    def _online_training(self):
        """Perform online training based on prediction error"""
        try:
            last_sub = self.last_mcs_hist[-(self.pred_len + self.delta):-self.delta]
            target_sub = self.target[-self.pred_len:]
            pred_sub = self.prediction[-(self.pred_len + self.delta):-self.delta]
            
            if len(last_sub) == len(target_sub) and len(pred_sub) == len(target_sub):
                err_t = self._weighted_mse(last_sub.flatten(), target_sub.flatten())
                err_p = self._weighted_mse(pred_sub.flatten(), target_sub.flatten())
                
                if err_p <= err_t * self.alpha:
                    pass  # Prediction is good
                else:
                    # Use fallback
                    self.corrected_predict[-1] = self.last_mcs_hist[-1]
                    
                    # Train if error is significant
                    if err_t > 1e-6 and len(self.train_data) >= self.delta + self.batch_size:
                        x_train = self.train_data[-self.delta - self.batch_size:-self.delta]
                        y_train = self.target[-self.batch_size:]
                        
                        x_t = torch.FloatTensor(x_train)
                        y_t = torch.FloatTensor(y_train)
                        
                        self.optimizer.zero_grad()
                        out = self.model(x_t)
                        loss = self.criterion(out, y_t)
                        loss.backward()
                        self.optimizer.step()
                        
                        if self.step_count % 100 == 0:
                            log(f"Training: step={self.step_count} loss={loss.item():.4f}")
        except Exception as e:
            log(f"Training error: {e}")
    
    @staticmethod
    def _weighted_mse(y_pred, y_true):
        n = len(y_pred)
        if n == 0:
            return 0.0
        weights = (1 + np.arange(n)) / n
        return np.mean(((y_pred - y_true) ** 2) * weights)
    
    @staticmethod
    def _sinr_to_mcs(sinr):
        """
        Map SINR to MCS using lookup table.
        Thresholds from uavlink project's link-level simulation (ns-3)
        for BLER target = 0.1, 
        [1] J. Yu and H. Jiang, “ns3-uavlink: AI-Driven Dynamic MCS Scheduling for U2U Sidelink Communication,” IEEE VTC.
        """
        thresholds = [
            -6.02, -4.14, -2.05, -0.03, 1.99, 7.03, 9.93, 11.01, 11.95, 12.09,
            13.10, 15.12, 16.07, 19.03, 19.10, 21.06, 21.13, 23.02, 23.96,
            24.09, 28.07, 31.10, 35.14
        ]
        for i, thr in enumerate(reversed(thresholds)):
            if sinr >= thr:
                return 22 - i
        return 0


if HAS_TORCH:
    class LSTMModel(nn.Module):
        """LSTM model for MCS prediction"""
        def __init__(self, input_size, hidden_size, output_size):
            super().__init__()
            self.dense1 = nn.Linear(input_size, 30)
            self.swish = nn.SiLU()
            self.lstm = nn.LSTM(30, hidden_size, batch_first=True)
            self.fc = nn.Linear(hidden_size, output_size)
        
        def forward(self, x):
            x = self.swish(self.dense1(x))
            out, _ = self.lstm(x)
            out = self.fc(out[:, -1, :])
            return out


# ==================== Main Loop ====================

def main():
    log("Starting U2U-MCS LSTM Agent")
    agent = None
    schema_hash = None

    while True:
        msg = read_message()
        if msg is None:
            break
        
        msg_type = msg.get('type')
        
        try:
            if msg_type == 'hello':
                # Handshake
                data = msg.get('data', {})
                schema_hash = data.get('schemaHash', '')
                
                log(f"Handshake received, schema: {schema_hash[:16]}...")
                
                agent = LSTMAgent(
                    obs_dim=data.get('observationSpace', {}).get('shape', [7])[0],
                    act_dim=data.get('actionSpace', {}).get('n', 23)
                )
                
                resp = create_response(msg, {
                    "accepted": True,
                    "schemaHash": schema_hash,
                    "agentVersion": "1.0.0",
                    "agentName": "u2u-mcs-lstm"
                }, 'hello_ack')  # Use hello_ack to match ProtocolHandler
                send_message(resp)
                log("Handshake complete")
            
            elif msg_type == 'reset':
                if agent is None:
                    send_message(create_error_response(
                        msg, ERROR_CODES['INTERNAL_ERROR'],
                        "Agent not initialized"
                    ))
                else:
                    action = agent.reset()
                    resp = create_response(msg, {"action": action}, 'action')
                    send_message(resp)
            
            elif msg_type == 'step':
                if agent is None:
                    send_message(create_error_response(
                        msg, ERROR_CODES['INTERNAL_ERROR'],
                        "Agent not initialized"
                    ))
                else:
                    data = msg.get('data', {})
                    action = agent.select_action(
                        data.get('observation', []),
                        data.get('reward', 0),
                        data.get('done', False)
                    )
                    resp = create_response(msg, {"action": action}, 'action')
                    send_message(resp)
            
            elif msg_type == 'close':
                log("Close received, shutting down")
                break
            
            else:
                send_message(create_error_response(
                    msg, ERROR_CODES['PROTOCOL_ERROR'],
                    f"Unknown message type: {msg_type}"
                ))
        
        except Exception as e:
            log(f"Error handling message: {e}")
            traceback.print_exc(file=sys.stderr)
            send_message(create_error_response(
                msg, ERROR_CODES['INTERNAL_ERROR'],
                str(e), traceback.format_exc()
            ))
    
    log("Agent terminated")


if __name__ == "__main__":
    main()
