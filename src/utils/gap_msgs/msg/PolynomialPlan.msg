std_msgs/Header header
string trajectory_id
string scene_id
gap_msgs/FlightModel model
# Each piece: 3 rows (x,y,z), 6 coefficients in ascending powers of local time.
float64[] durations
float64[] coefficients
