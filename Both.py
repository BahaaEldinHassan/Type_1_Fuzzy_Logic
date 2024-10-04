import rclpy
import math
from itertools import product
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf2_ros import TransformRegistration
from rclpy.qos import QoSProfile, ReliabilityPolicy

# Define input space for fuzzy logic in terms of distance (near, medium, far)
near = [0.0, 0.25, 0.5]
med = [0.25, 0.5, 0.75]
far = [0.5, 0.75, 1.0]
middles = [0.25, 0.5, 0.75]
# Define output spaces for linear and angular velocities
slow = 0.07
medium = 0.09
fast = 0.2
rite = -0.35
forward = 0.0
left = 0.4

# Define a rule base for right edge following logic
rule_base_right_edge = {
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
# Membership dictionaries for right edge following
mffrs = {"frs_near_deg": 0.0, "frs_med_deg": 0.0, "frs_far_deg": 0.0}
mfbrs = {"brs_near_deg": 0.0, "brs_med_deg": 0.0, "brs_far_deg": 0.0}

# Define output spaces for another set of fuzzy logic (OA - Obstacle Avoidance)
slow_OA = 0.06
medium_OA = 0.1
fast_OA = 0.2
rite_OA = -0.5
forward_OA = 0.00001
left_OA = 0.6

# Define a rule base for obstacle avoidance logic
rule_base_OA = {
    ("f1_near_deg", "f2_near_deg"): (slow_OA, left_OA),
    ("f1_near_deg", "f2_med_deg"): (slow_OA, rite_OA),
    ("f1_near_deg", "f2_far_deg"): (slow_OA, rite_OA),
    ("f1_med_deg", "f2_near_deg"): (slow_OA, left_OA),
    ("f1_med_deg", "f2_med_deg"): (medium_OA, forward_OA),
    ("f1_med_deg", "f2_far_deg"): (medium_OA, rite_OA),
    ("f1_far_deg", "f2_near_deg"): (slow_OA, left_OA),
    ("f1_far_deg", "f2_med_deg"): (medium_OA, left_OA),
    ("f1_far_deg", "f2_far_deg"): (fast_OA, forward_OA),
}
# Membership functions for obstacle avoidance
mff1 = {"f1_near_deg": 0.0, "f1_med_deg": 0.0, "f1_far_deg": 0.0}
mff2 = {"f2_near_deg": 0.0, "f2_med_deg": 0.0, "f2_far_deg": 0.0}


# Fuzzification function for obstacle avoidance
def fuzzification_OA():
    # Logic for converting crisp inputs to fuzzy values until defuzzification.
    global regions_, rule_base_OA
    f1 = regions_['front1']
    f2 = regions_['front2']
    mff1 = calculate_membership_OA(f1, middles, 0)
    print(mff1)
    mff2 = calculate_membership_OA(f2, middles, 1)
    print(mff2)
    mff1_new, mff2_new = {}, {}
    for k, v in mff1.items():
        if v != 0:
            mff1_new[k] = v
    print("mff1", mff1_new)
    for k, v in mff2.items():
        if v != 0:
            mff2_new[k] = v
    print("mff2", mff2_new)
    firing_rules_membership_output = [rule_base_OA[k] for k in list(product(mff1_new, mff2_new))]
    firing_rules_values = [min(t) for t in list(product(mff1_new.values(), mff2_new.values()))]
    x, z = 0.0, 0.0
    for i, t in enumerate(firing_rules_membership_output):
        x += firing_rules_values[i] * t[0]
        z += firing_rules_values[i] * t[1]
    x_linear = x / sum(firing_rules_values)
    z_angular = z / sum(firing_rules_values)
    print("x_linear,z_angular", x_linear, z_angular)
    return x_linear, z_angular


# Calculate membership degrees for obstacle avoidance
def calculate_membership_OA(value, middles, fuzzy_set):
    # Calculate the degree of membership for f2 a value in a fuzzy set
    output = {}
    global regions_, rules
    f1 = regions_['front1']
    f2 = regions_['front2']
    if (fuzzy_set == 0):
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
    print("OA_calc", output)
    return output


# Fuzzification function for right edge following
def fuzzification_right_edge():
    global regions_, rule_base_right_edge
    frs = regions_['fright']
    brs = regions_['bright']
    mffrs = calculate_membership_right_edge(frs, middles, 0)
    print(mffrs)
    mfbrs = calculate_membership_right_edge(brs, middles, 1)
    print(mfbrs)
    mffrs_new, mfbrs_new = {}, {}
    for k, v in mffrs.items():
        if v != 0:
            mffrs_new[k] = v
    print("mffrs", mffrs_new)
    for k, v in mfbrs.items():
        if v != 0:
            mfbrs_new[k] = v
    print("mfbrs", mfbrs_new)
    print(type(product(mffrs_new, mfbrs_new)))
    firing_rules_membership_output = [rule_base_right_edge[k] for k in list(product(mffrs_new, mfbrs_new))]
    print(type(firing_rules_membership_output))
    print("hello3", firing_rules_membership_output)
    firing_rules_values = [min(t) for t in list(product(mffrs_new.values(), mfbrs_new.values()))]
    x, z = 0.0, 0.0
    for i, t in enumerate(firing_rules_membership_output):
        x += firing_rules_values[i] * t[0]
        z += firing_rules_values[i] * t[1]
    x_linear = x / sum(firing_rules_values)
    z_angular = z / sum(firing_rules_values)
    return x_linear, z_angular


# Calculate membership degrees for right edge following
def calculate_membership_right_edge(value, middles, fuzzy_set=0):
    # Calculate the degree of membership for brs and frs values in a fuzzy set
    output = {}
    global regions_
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


# Combined fuzzification logic
def fuzzification_both():
    global regions_
    frs = regions_['fright']
    brs = regions_['bright']
    f1 = regions_['front1']
    f2 = regions_['front2']
    # print("f1_22:", f1)
    D1 = min(frs, brs)
    D2 = min(f1, f2)
    output = []
    membership_1, membership_2 = calculate_membership_both(D1, D2)
    # print("here")
    linear_right_edge, angular_right_edge = fuzzification_right_edge()
    print(f"fuzzyright_edge:{linear_right_edge}, {angular_right_edge}\n")
    # print("\n", linear_right_edge)
    # print ("here2")
    linear_OA, angular_OA = fuzzification_OA()
    print(f"fuzzyright_OA:{linear_OA}, {angular_OA}\n")

    linear_output = ((membership_1 * linear_right_edge) + (membership_2 * linear_OA)) / (membership_1 + membership_2)
    angular_output = ((membership_1 * angular_right_edge) + (membership_2 * angular_OA)) / (membership_1 + membership_2)
    # Combine outputs from both fuzzification systems
    output.append(linear_output)
    output.append(angular_output)
    print("\n summed_output", output)
    return output


# Calculate membership for combined logic
def calculate_membership_both(D1, D2):
    output = []
    membership_d1 = 0.0000001
    membership_d2 = 0.0000001
    if (D2 < 0.2):
        membership_d2 = 1
    if (D2 > 0.2 and D2 < 0.5):
        membership_d2 = (0.5 - D2) / (0.5 - 0.45)
    if (D1 > 0.75):
        membership_d1 = 1
    if (D1 < 0.75 and D1 > 0.65):
        membership_d1 = membership_d1 = (D1 - 0) / (0.75 - 0)
    output.append(membership_d1)
    output.append(membership_d2)
    print("output of both", output)
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
        'front1': find_nearest(msg.ranges[0:30]),
        'front2': find_nearest(msg.ranges[315:340]),
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
    return min(min(f_list, default=10), 1)


# Basic movement method
def movement():
    global regions_, mynode_
    regions = regions_

    # create an object of twist class, used to express the linear and angular velocity of the turtlebot
    msg = Twist()
    linear_output, angular_output = fuzzification_both()
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
