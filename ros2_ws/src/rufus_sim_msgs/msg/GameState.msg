# Aggregate game state, published by the game engine each tick.
# active_predicates lists termination predicate IDs that are currently
# satisfied but still inside their dwell-time window.
std_msgs/Header header
string episode_id
builtin_interfaces/Duration sim_time
AgentState[] agents
string[] active_predicates
