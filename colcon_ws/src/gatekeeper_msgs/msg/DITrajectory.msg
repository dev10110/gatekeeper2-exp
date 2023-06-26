std_msgs/Header header

# from the start time in the header, construct a trajectory
float64 dt # timestep between these messages
geometry_msgs/Point[] pos
geometry_msgs/Vector3[] vel
geometry_msgs/Vector3[] acc
float64[] yaw
