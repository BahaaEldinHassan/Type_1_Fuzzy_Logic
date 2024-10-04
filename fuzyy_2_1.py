import rclpy
import math
from itertools import product
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf2_ros import TransformRegistration
from rclpy.qos import QoSProfile, ReliabilityPolicy

# Define the input space for fuzzy logic, categorizing distances into 'near', 'medium', and 'far'.
near = [0.0, 0.25, 0.5]  # 'Near' distance range
med = [0.25, 0.5, 0.75]  # 'Medium' distance range
far = [0.5, 0.75, 1.0]  # 'Far' distance range
middles = [0.25, 0.5, 0.75]  # Middle values for each range, used in membership calculations

# Define the output space for right edge following in terms of linear and angular velocities.
slow = 0.1  # Slow linear speed
medium = 0.15  # Medium linear speed
fast = 0.2  # Fast linear speed
rite = -0.35  # Angular speed for turning right
forward = 0.0  # Zero angular speed, representing moving forward
left = 0.35  # Angular speed for turning left

# Rule base for the fuzzy logic controller, mapping combinations of distance categories to specific linear and angular speeds.
rule_base_right_edge = {
    # Each tuple key represents a combination of distance categories for two sensors (frs - front right sensor, brs - back right sensor).
    # The corresponding value is a tuple defining linear and angular speeds.

    ("frs_near_deg", "brs_near_deg"): (slow, left),
    ("frs_near_deg", "brs_med_deg"): (medium, left),
    ("frs_near_deg", "brs_far_deg"): (medium, left),
    ("frs_med_deg", "brs_near_deg"): (medium, forward),
    ("frs_med_deg", "brs_med_deg"): (medium, forward),
    ("frs_med_deg", "brs_far_deg"): (medium, rite),
    ("frs_far_deg", "brs_near_deg"): (medium, rite),
    ("frs_far_deg", "brs_med_deg"): (medium, rite),
    ("frs_far_deg", "brs_far_deg"): (fast, rite),
}
# Initializing membership functions for front right sensor (frs) and back right sensor (brs).
# These will be used to hold the degree of membership in each distance category (near, medium, far).
mffrs = {"frs_near_deg": 0.0, "frs_med_deg": 0.0, "frs_far_deg": 0.0}
mfbrs = {"brs_near_deg": 0.0, "brs_med_deg": 0.0, "brs_far_deg": 0.0}


def fuzzification_right_edge():
    global regions_, rule_base_right_edge
    # Retrieve sensor readings for the front and back right sensors
    frs = regions_['fright']
    brs = regions_['bright']

    # Calculate the degree of membership for each sensor reading in each distance category
    mffrs = calculate_membership_right_edge(frs, middles, 0)
    mfbrs = calculate_membership_right_edge(brs, middles, 1)

    # Filter out categories with zero membership for both sensors
    mffrs_new, mfbrs_new = {}, {}
    for k, v in mffrs.items():
        if v != 0:
            mffrs_new[k] = v
    for k, v in mfbrs.items():
        if v != 0:
            mfbrs_new[k] = v

    # Compute the output of the firing rules
    firing_rules_membership_output = [rule_base_right_edge[k] for k in list(product(mffrs_new, mfbrs_new))]
    firing_rules_values = [min(t) for t in list(product(mffrs_new.values(), mfbrs_new.values()))]

    # Calculate the weighted average of the outputs, determining the final linear and angular speeds
    x, z = 0.0, 0.0
    for i, t in enumerate(firing_rules_membership_output):
        x += firing_rules_values[i] * t[0]
        z += firing_rules_values[i] * t[1]
    x_linear = x / sum(firing_rules_values)
    z_angular = z / sum(firing_rules_values)

    return x_linear, z_angular


def calculate_membership_right_edge(value, middles, fuzzy_set):
    # This function calculates the degree of membership of a given value in the fuzzy sets 'near', 'medium', and 'far'.
    output = {}
    global regions_, rules
    # Based on the fuzzy_set flag, populate the output dictionary with the membership values.
    # The calculations are based on the position of the value in the input space (near, medium, far).
    frs = regions_['fright']
    brs = regions_['bright']
    if (fuzzy_set == 0):
        output = {"frs_near_deg": 0, "frs_med_deg": 0, "frs_far_deg": 0}
        if value <= middles[0]:
            output["frs_near_deg"] = 1
        elif middles[0] < value <= middles[1]:
            output["frs_near_deg"] = (middles[1] - value) / (middles[1] - middles[0])
            output["frs_med_deg"] = (value - middles[0]) / (middles[1] - middles[0])
        elif middles[1] < value <= middles[2]:
            output["frs_med_deg"] = (middles[2] - value) / (middles[2] - middles[1])
            output["frs_far_deg"] = (value - middles[1]) / (middles[2] - middles[1])
        else:
            output["frs_far_deg"] = 1
    else:
        output = {"brs_near_deg": 0, "brs_med_deg": 0, "brs_far_deg": 0}
        if value <= middles[0]:
            output["brs_near_deg"] = 1
        elif middles[0] < value <= middles[1]:
            output["brs_near_deg"] = (middles[1] - value) / (middles[1] - middles[0])
            output["brs_med_deg"] = (value - middles[0]) / (middles[1] - middles[0])
        elif middles[1] < value <= middles[2]:
            output["brs_med_deg"] = (middles[2] - value) / (middles[2] - middles[1])
            output["brs_far_deg"] = (value - middles[1]) / (middles[2] - middles[1])
        else:
            output["brs_far_deg"] = 1

    return output


# Rest of the code...


mynode_ = None
pub_ = None
regions_ = {
    'right': 0,
    'fright': 0,
    'front1': 0,
    'front2': 0,
    'fleft': 0,
    'left': 0,
    'bright': 0,
}
twstmsg_ = None


# main function attached to timer callback
def timer_callback():
    global pub_, twstmsg_
    if (twstmsg_ != None):
        pub_.publish(twstmsg_)


def clbk_laser(msg):
    global regions_, twstmsg_

    regions_ = {
        # LIDAR readings are anti-clockwise
        'front1': find_nearest(msg.ranges[0:5]),
        'front2': find_nearest(msg.ranges[355:360]),
        'right': find_nearest(msg.ranges[265:275]),
        'fright': find_nearest(msg.ranges[300:355]),
        'fleft': find_nearest(msg.ranges[40:50]),
        'left': find_nearest(msg.ranges[85:95]),
        'bright': find_nearest(msg.ranges[210:270])
    }
    twstmsg_ = movement()


# Find nearest point
def find_nearest(list):
    f_list = filter(lambda item: item > 0.0, list)  # exclude zeros
    return min(min(f_list, default=10), 10)


# Basic movement method
def movement():
    global regions_, mynode_
    regions = regions_

    # create an object of twist class, used to express the linear and angular velocity of the turtlebot
    msg = Twist()
    linear_output, angular_output = fuzzification_right_edge()
    print("Linear Speed:", linear_output)
    print("Angular Speed:", angular_output)
    msg.linear.x = linear_output
    msg.angular.z = angular_output
    return msg


# used to stop the rosbot
def stop():
    global pub_
    msg = Twist()
    msg.angular.z = 0.0
    msg.linear.x = 0.0
    pub_.publish(msg)
    return msg


def main():
    global pub_, mynode_

    rclpy.init()
    mynode_ = rclpy.create_node('reading_laser')

    # define qos profile (the subscriber default 'reliability' is not compatible with robot publisher 'best effort')
    qos = QoSProfile(
        depth=10,
        reliability=ReliabilityPolicy.BEST_EFFORT,
    )

    # publisher for twist velocity messages (default qos depth 10)
    pub_ = mynode_.create_publisher(Twist, '/cmd_vel', 10)

    # subscribe to laser topic (with our qos)
    sub = mynode_.create_subscription(LaserScan, '/scan', clbk_laser, qos)

    # Configure timer
    timer_period = 0.2  # seconds
    timer = mynode_.create_timer(timer_period, timer_callback)

    # Run and handle keyboard interrupt (ctrl-c)
    try:
        rclpy.spin(mynode_)
    except KeyboardInterrupt:
        stop()  # stop the robot
    except:
        stop()  # stop the robot
    finally:
        # Clean up
        mynode_.destroy_timer(timer)
        mynode_.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
