# enemy state
std_msgs/Header header
int32 enemy_count
int32[] enemy_sectors
# position in grid, in ENU
geometry_msgs/Point32[] enemy_position
bool[] is_captured
