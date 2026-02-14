# Array of tracked targets

std_msgs/Header header

# All current tracks
TrackedTarget[] targets

# ID of highest priority target (for turret to track)
uint32 primary_target_id
