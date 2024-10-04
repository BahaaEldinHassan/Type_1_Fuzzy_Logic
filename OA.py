import rclpy
import math
from itertools import product
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf2_ros import TransformRegistration
from rclpy.qos import QoSProfile, ReliabilityPolicy

# Defining the input space for fuzzy logic, categorizing distances into 'near', 'medium', and 'far' ranges.
near = [0.0, 0.25, 0.5]  # 'Near' distance range
med = [0.25, 0.5, 0.75]  # 'Medium' distance range
far = [0.5, 0.75, 1.0]   # 'Far' distance range
middles = [0.25, 0.5, 0.75]  # Middle values for each range, used in membership calculations

# Defining the output space for the obstacle avoidance system in terms of linear and angular velocities.
slow_OA = 0.06   # Slow linear speed
medium_OA = 0.13 # Medium linear speed
fast_OA = 0.2    # Fast linear speed
rite_OA = -0.5   # Angular speed for turning right
forward_OA = 0.00001 # Almost zero angular speed, representing moving forward
left_OA = 0.5    # Angular speed for turning left

# Rule base for the fuzzy logic controller,
# mapping combinations of distance categories to specific linear and angular speeds.
rule_base_OA = {
    # Each tuple key represents a combination of distance categories for two front sensors (f1 and f2).
    # The corresponding value is a tuple defining linear and angular speeds.
    ("f1_near_deg", "f2_near_deg"): (slow_OA, left_OA),
    ("f1_near_deg", "f2_med_deg"): (slow_OA, rite_OA),
    ("f1_near_deg", "f2_far_deg"): (medium_OA, rite_OA),
    ("f1_med_deg", "f2_near_deg"): (slow_OA, left_OA),
    ("f1_med_deg", "f2_med_deg"): (medium_OA, forward_OA),
    ("f1_med_deg", "f2_far_deg"): (medium_OA, rite_OA),
    ("f1_far_deg", "f2_near_deg"): (slow_OA, left_OA),
    ("f1_far_deg", "f2_med_deg"): (medium_OA, left_OA),
    ("f1_far_deg", "f2_far_deg"): (fast_OA, forward_OA),
}
# Initializing membership functions for front sensors (f1 and f2).
# These will be used to hold the degree of membership in each distance category (near, medium, far).
mff1 = {"f1_near_deg": 0.0, "f1_med_deg": 0.0, "f1_far_deg": 0.0}
mff2 = {"f2_near_deg": 0.0, "f2_med_deg": 0.0, "f2_far_deg": 0.0}


# Fuzzing Crisp Input
def fuzzification_OA():
    global regions_,rule_base_OA
    # Retrieve sensor readings for the two front sensors
    f1 = regions_['front1']
    f2 = regions_['front2']
    # Calculate the degree of membership for each sensor reading in each distance category
    mff1 = calculate_membership_OA(f1, middles,0)
    print(mff1)
    mff2 = calculate_membership_OA(f2, middles,1)
    print(mff2)
    mff1_new, mff2_new = {}, {}
    # Filter out categories with zero membership for both sensors
    for k, v in mff1.items():
        if v != 0:
            mff1_new[k] = v
    for k, v in mff2.items():
        if v != 0:
            mff2_new[k] = v
    # Compute the output of the firing rules
    firing_rules_membership_output = [rule_base_OA[k] for k in list(product(mff1_new, mff2_new))]
    firing_rules_values = [min(t) for t in list(product(mff1_new.values(), mff2_new.values()))]
    x, z = 0.0, 0.0
    # Calculate the weighted average of the outputs, determining the final linear and angular speeds
    for i, t in enumerate(firing_rules_membership_output):

        x += firing_rules_values[i]*t[0]
        z += firing_rules_values[i]*t[1]
    x_linear = x/sum(firing_rules_values)
    z_angular = z/sum(firing_rules_values)
    return x_linear,z_angular


def calculate_membership_OA(value, middles, fuzzy_set):
    # Calculate the degree of membership for f2 a value in a fuzzy set
    output={}
    global regions_, rules
    # Based on the fuzzy_set flag, populate the output dictionary with the membership values.
    # The calculations are based on the position of the value in the input space (near, medium, far).
    f1 = regions_['front1']
    f2 = regions_['front2']
    if (fuzzy_set == 0 ):
        output = {"f1_near_deg": 0, "f1_med_deg": 0, "f1_far_deg": 0}
        if value <= middles[0]:
            output["f1_near_deg"] = 1
        elif middles[0] < value <= middles[1]:
            output["f1_near_deg"] = (middles[1] - value) / (middles[1] - middles[0])
            output["f1_med_deg"] = (value - middles[0]) / (middles[1] - middles[0])
        elif middles[1] < value <= middles[2]:
            output["f1_med_deg"] = (middles[2] - value) / (middles[2] - middles[1])
            output["f1_far_deg"] = (value - middles[1]) / (middles[2] - middles[1])
        else:
            output["f1_far_deg"] = 1
    else:
        output = {"f2_near_deg": 0, "f2_med_deg": 0, "f2_far_deg": 0}
        if value <= middles[0]:
            output["f2_near_deg"] = 1
        elif middles[0] < value <= middles[1]:
            output["f2_near_deg"] = (middles[1] - value) / (middles[1] - middles[0])
            output["f2_med_deg"] = (value - middles[0]) / (middles[1] - middles[0])
        elif middles[1] < value <= middles[2]:
            output["f2_med_deg"] = (middles[2] - value) / (middles[2] - middles[1])
            output["f2_far_deg"] = (value - middles[1]) / (middles[2] - middles[1])
        else:
            output["f2_far_deg"] = 1

    return output
    




# Rest of the code...


mynode_ = None
pub_ = None
regions_ = {
    'right': 0,
    'fright': 0,
    'front1': 0,
    'front2': 0,
    'fleft_OA': 0,
    'left_OA': 0,
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
        'front1': find_nearest(msg.ranges[0:60]),
        'front2': find_nearest(msg.ranges[315:360]),
        'right': find_nearest(msg.ranges[265:275]),
        'fright': find_nearest(msg.ranges[300:355]),
        'fleft_OA': find_nearest(msg.ranges[40:50]),
        'left_OA': find_nearest(msg.ranges[85:95]),
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
    linear_output, angular_output = fuzzification_OA()
    print("Linear Speed:", linear_output)
    print("Angular Speed:", angular_output)
    msg.linear.x = linear_output
    msg.angular.z = angular_output
    return msg




#used to stop the rosbot
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

