# Pairwise distances computed from optimized relative structure positions.
std_msgs/Header header

uint8 group_id
string leader_name

# Each index k describes one pair:
#   robot_a_names[k] <-> robot_b_names[k] has distance distances[k].
string[] robot_a_names
string[] robot_b_names
float64[] distances
