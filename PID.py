import rclpy
import math

from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf2_ros import TransformRegistration
from rclpy.qos import QoSProfile, ReliabilityPolicy

# Initialize global variables
mynode_ = None  # Node object for ROS communication
pub_ = None  # Publisher object for sending twist messages
regions_ = {  # Dictionary to store nearest distances in various directions from LIDAR data
    'right': 0,
    'fright': 0,
    'front1': 0,
    'front2': 0,
    'fleft': 0,
    'left': 0,
}
twstmsg_ = None  # Variable to store the twist message for robot movement   # Use as backup in case of emergency
Kp = 0.5  # Proportional gain for PID controller                            # kp = 0.5, 0,5
Ki = 0.02  # Integral gain for PID controller                               # ki = 0.000002,  0.02003
Kd = 0.005  # Derivative gain for PID controller                            # kd = 0.4000233, 0.002004
e_prev = 0  # Previous error for derivative calculation in PID              # eprev = 0     , 0
e_i = 0.7  # Integral error for PID                                         # ei = 0.7      , 0.7
e_d = 0.400004  # Derivative error for PID                                  # e_d = 0.400004, 0.40004


# Function called periodically by a timer
def timer_callback():
    global pub_, twstmsg_
    if (twstmsg_ != None):
        pub_.publish(twstmsg_)


# Callback function for LIDAR data processing
def clbk_laser(msg):
    global regions_, twstmsg_

    # Updating regions with nearest distances in specified directions
    regions_ = {
        'front1': find_nearest(msg.ranges[0:5]),
        'front2': find_nearest(msg.ranges[355:360]),
        'right': find_nearest(msg.ranges[235:275]),
        'fright': find_nearest(msg.ranges[310:320]),
        'fleft': find_nearest(msg.ranges[40:50]),
        'left': find_nearest(msg.ranges[85:95])
    }
    twstmsg_ = movement()


# Finds nearest point in a list, excluding zero values
def find_nearest(list):
    f_list = filter(lambda item: item > 0.0, list)
    return min(min(f_list, default=10), 10)


# PID controller for rightward motion
def PID_right(e):
    global e_prev, e_i, e_d, Kp, Ki, Kd
    if (abs(e_i + e) < 0.2):
        e_i = e_i + e
    e_d = e - e_prev
    output = Kp * e + Ki * e_i + Kd * e_d
    print(output)
    e_prev = e
    return output


# Determines robot movement based on sensor data
def movement():
    global regions_, mynode_
    regions = regions_
    desired_distance = 0.4
    # Creating twist message for linear and angular velocity
    msg = Twist()
    error = desired_distance - regions_['right']
    msg.linear.x = 0.1
    print("PID_output: ", PID_right(error))
    msg.angular.z = PID_right(error)
    return msg


# Stops the robot by publishing zero velocities
def stop():
    global pub_
    msg = Twist()
    msg.angular.z = 0.0
    msg.linear.x = 0.0
    pub_.publish(msg)


# Main function to initialize node, publishers, subscribers, and timer
def main():
    global pub_, mynode_

    rclpy.init()
    mynode_ = rclpy.create_node('reading_laser')

    # Define Quality of Service (QoS) profile for subscriber
    qos = QoSProfile(
        depth=10,
        reliability=ReliabilityPolicy.BEST_EFFORT,
    )

    # Initialize publisher and subscriber with appropriate topics and QoS
    pub_ = mynode_.create_publisher(Twist, '/cmd_vel', 10)
    sub = mynode_.create_subscription(LaserScan, '/scan', clbk_laser, qos)

    # Set up a timer for periodic callback execution
    timer_period = 0.2  # seconds
    timer = mynode_.create_timer(timer_period, timer_callback)

    # Main loop with exception handling
    try:
        rclpy.spin(mynode_)
    except KeyboardInterrupt:
        stop()  # Stop the robot on keyboard interrupt
    except:
        stop()  # Stop the robot on other exceptions
    finally:
        # Clean up resources
        mynode_.destroy_timer(timer)
        mynode_.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
