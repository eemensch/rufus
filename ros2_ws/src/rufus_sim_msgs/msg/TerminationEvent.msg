# Emitted once when an episode terminates.
# predicate_id is the ID of the firing predicate (empty for timeout).
# outcome is a free-form string; the OUTCOME_* constants are the
# conventional values.

string OUTCOME_PURSUER_WIN="pursuer_win"
string OUTCOME_EVADER_WIN="evader_win"
string OUTCOME_DRAW="draw"
string OUTCOME_TIMEOUT="timeout"

std_msgs/Header header
string episode_id
builtin_interfaces/Duration sim_time
string predicate_id
string outcome
GameState terminal_state
