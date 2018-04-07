std_msgs/Header header

int32 my_id

int32 my_current_sector
int32 my_next_sector

geometry_msgs/Point32 my_current_position
geometry_msgs/Point32 my_current_local_position
geometry_msgs/Point32 my_next_position
geometry_msgs/Point32 my_next_local_position

# set of sensed neighbors; others are ignored
# length of this message is as long as the sensed neighbors only
int32[] sensed_neighbors

# contains the sensed neighbors; non neighbors are set to zero
# the length of this msg is equal to the number of agents in the team
int32[] sensed_neighbors_full_msg

# Predicted locations of agents
int32[] estimated_neighbors_next_locations

# sensed atackers locations
int32[] sensed_attackers_locaitons

# predicted attackers locations from this defender's perspective
int32[] predicted_attackers_locations

# in milli seconds
float32 execution_time_ms
