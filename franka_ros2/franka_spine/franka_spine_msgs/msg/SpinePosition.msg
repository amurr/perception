# Periodic spine position sample published on ~/position.
# Subscribe to this instead of polling the GetPosition service in a loop.
std_msgs/Header header
float64 position  # Last known position [m]
bool valid        # False when the sample is stale because the REST read failed
