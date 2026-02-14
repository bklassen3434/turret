# Fused target track from sensor fusion
# Combines camera and radar data via Kalman filter

std_msgs/Header header

# Unique track ID
uint32 track_id

# Track state
uint8 STATE_TENTATIVE = 0    # New track, not yet confirmed
uint8 STATE_CONFIRMED = 1    # Confirmed track
uint8 STATE_COASTING = 2     # Lost sensor contact, predicting
uint8 STATE_LOST = 3         # Track dropped
uint8 state

# Position in turret frame (meters)
geometry_msgs/Point position

# Velocity in turret frame (m/s)
geometry_msgs/Vector3 velocity

# Position covariance (3x3 matrix, row-major)
float64[9] position_covariance

# Velocity covariance (3x3 matrix, row-major)
float64[9] velocity_covariance

# Bearing to target (for turret control)
float64 azimuth    # radians
float64 elevation  # radians

# Range to target (meters)
float64 range

# Sensor contribution flags
bool camera_contributing
bool radar_contributing

# Classification from camera (if available)
string classification

# Track quality / confidence [0.0 - 1.0]
float64 confidence

# Time since last measurement update (seconds)
float64 time_since_update
