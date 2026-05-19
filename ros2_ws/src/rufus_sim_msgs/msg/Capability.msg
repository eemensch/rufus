# Latched, episode-level descriptor of the velocity-command envelope
# for an agent, derived from the underlying autopilot parameters at
# setup. Strategies use it for a-priori feasibility checks; the
# adapter still enforces limits at runtime via SaturationFlags.

std_msgs/Header header
string  agent_id
uint8   platform              # use AgentState.PLATFORM_*

# Speed envelope (m/s).
float64 v_max
float64 v_min                 # rover/quad: -v_max (reverse allowed);
                              # fixed-wing: >0 (cannot stop in air)

# Vertical envelope (m/s, positive up).
float64 vz_max_up             # 0 for rover
float64 vz_max_down           # 0 for rover

# Heading and turn limits.
float64 yaw_rate_max          # rad/s, body yaw rate (rover skid, quad)
float64 lateral_accel_max     # m/s^2, drives velocity-vector turn rate
                              # via psi_dot_vel(V) = lateral_accel_max / V
float64 bank_angle_max        # rad, informational (fixed-wing, quad)
float64 climb_angle_max       # rad, informational (fixed-wing)
float64 min_turn_radius       # m, rover Dubins-car coupling:
                              # |yaw_rate| <= |v| / min_turn_radius,
                              # from rover TURN_RADIUS. 0 = not
                              # radius-limited: quad is holonomic;
                              # fixed-wing uses the speed-coupled
                              # lateral_accel_max (turn radius
                              # V^2 / lateral_accel_max is not a
                              # single constant).

# Audit trail: JSON-serialised record of source parameter names and
# values queried from the autopilot at episode setup.
string  source
