# Camera-based detection message
# Provides bearing (angle) to target but no range

std_msgs/Header header

# Detection ID (for tracking across frames)
uint32 detection_id

# Bearing angles from camera boresight (radians)
float64 azimuth    # Horizontal angle (positive = right)
float64 elevation  # Vertical angle (positive = up)

# Pixel coordinates in image
float64 pixel_x
float64 pixel_y

# Bounding box (pixels)
float64 bbox_width
float64 bbox_height

# Detection confidence [0.0 - 1.0]
float64 confidence

# Target classification (e.g., "red_ball", "aruco_marker")
string classification
