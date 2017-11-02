std_msgs/Header header

int32 my_id

int32 my_current_sector
int32 my_next_sector

geometry_msgs/Point32 my_current_position
geometry_msgs/Point32 my_current_local_position
geometry_msgs/Point32 my_next_position
geometry_msgs/Point32 my_next_local_position

int32[] sensed_neighbors

# sensed atackers locations
int32[] sensed_attackers_locaitons

# predicted attackers locations from this defender's perspective
int32[] predicted_attackers_locations

# in milli seconds
float32 execution_time_ms
