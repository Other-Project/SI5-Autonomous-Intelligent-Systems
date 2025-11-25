from geometry_msgs.msg import Twist

def wheels_to_twist(v_l: float, v_r: float, wheel_separation: float) -> Twist:
    """
    Convert left/right wheel linear speeds (m/s) to a geometry_msgs/Twist.
    v_l: velocity left wheel (m/s)
    v_r: velocity right wheel (m/s)
    wheel_separation: distance between wheels (m)
    Returns: Twist message with linear.x and angular.z set.
    """
    cmd = Twist()
    cmd.linear.x = (v_r + v_l) / 2.0
    # angular z = (v_r - v_l) / track_width
    if wheel_separation == 0:
        cmd.angular.z = 0.0
    else:
        cmd.angular.z = (v_r - v_l) / wheel_separation
    return cmd

def twist_to_wheels(linear_x: float, angular_z: float, wheel_separation: float):
    """
    Convert Twist components to wheel linear speeds (v_l, v_r).
    """
    v_r = linear_x + 0.5 * angular_z * wheel_separation
    v_l = linear_x - 0.5 * angular_z * wheel_separation
    return v_l, v_r

