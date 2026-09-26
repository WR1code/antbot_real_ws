# Simplified planar source path before MoveIt expands it into a joint trajectory.
std_msgs/Header header
string drawing_type
string source
geometry_msgs/Point[] points
# Index in points at which each disconnected pen-down stroke begins.
uint32[] stroke_start_indices
# Distance from gripper_tcp to the physical tip along local -Z.
float64 pen_length_m
# Pen-tip travel towards the arm between disconnected strokes.
float64 pen_lift_m
