# Radar-based detection message
# Provides range and range-rate but lower angular resolution

std_msgs/Header header

# Detection ID
uint32 detection_id

# Range to target (meters)
float64 range

# Range rate / radial velocity (m/s, positive = moving away)
float64 range_rate

# Bearing angles (radians) - lower resolution than camera
float64 azimuth
float64 elevation

# Signal strength / SNR (dB)
float64 snr

# Detection confidence [0.0 - 1.0]
float64 confidence
