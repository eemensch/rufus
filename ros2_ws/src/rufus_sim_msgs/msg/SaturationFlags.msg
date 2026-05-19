# Per-agent saturation indicators on the most recent commanded velocity.
# Each flag is true when the corresponding platform limit was active when
# the latest cmd_vel was projected onto the feasible setpoint.
bool linear_velocity
bool angular_velocity
bool airspeed
bool turn_rate
bool climb_rate
