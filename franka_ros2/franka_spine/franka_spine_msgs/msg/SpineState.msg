# Periodic spine state sample published on ~/state.
# Subscribe to this instead of polling the GetSpineState service in a loop.
# Possible states: NotReadyToSwitchOn, SwitchedOff, ReadyToSwitchOn,
#   MotorOff, SwitchedOn, QuickStopActive, FaultReactionActive, Fault, Unknown.
std_msgs/Header header
string state              # Current spine state
string error_code         # Error code (empty when no error)
string error_description  # Human-readable error description
bool valid                # False when the sample is stale because the REST read failed
