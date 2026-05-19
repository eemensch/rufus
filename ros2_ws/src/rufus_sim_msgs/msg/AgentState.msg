# Per-agent state snapshot.
# Pose is in the ENU world frame (header.frame_id, typically "map").
# Twist is in the agent body frame (the named convention is documented
# per platform in rufus_sim_adapters).

uint8 ROLE_NEUTRAL=0
uint8 ROLE_PURSUER=1
uint8 ROLE_EVADER=2

uint8 PLATFORM_ROVER=0
uint8 PLATFORM_QUADROTOR=1
uint8 PLATFORM_FIXED_WING=2

std_msgs/Header header
string agent_id
uint8 role
uint8 platform
geometry_msgs/Pose pose
geometry_msgs/Twist twist
SaturationFlags saturation
