
import rclpy
import math
import numpy as np
import os
import sys

movement_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..', 'Movement')
sys.path.append(movement_path)

from FuzzyLogicSystem import Fuzzer, FuzzyController

from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf2_ros import TransformRegistration
from rclpy.qos import QoSProfile, ReliabilityPolicy

current_dir = os.path.dirname(os.path.abspath(__file__))

# re_fuzz
re_json_path = os.path.join(current_dir, 'right_edge.json')
re_csv_path = os.path.join(current_dir, 'right_edge.csv')
re_inputs=['RFS', 'RBS']
re_outputs=['Speed','Direction']
re_model = Fuzzer(json_path= re_json_path,csv_path= re_csv_path, inputs=re_inputs, outputs=re_outputs)

# oa_fuzz
oa_json_path = os.path.join(current_dir, 'object_avoidance.json')
oa_csv_path = os.path.join(current_dir, 'object_avoidance.csv')
oa_inputs=['RFS', 'FS','LFS']
oa_outputs=['Speed','Direction']
oa_model = Fuzzer(json_path= oa_json_path, csv_path=oa_csv_path, inputs=oa_inputs, outputs=oa_outputs)

fuzzy_models = {
    're':{
        'model':re_model,
        'ranges': [0.25, 0.4, 1],
        'negate': []
        },
    'oa':{
        'model':oa_model,
        'ranges': [0.0, 0.25, 0.4],
        'negate': ['Direction']
        }
}
outputs = {'Speed', 'Direction'}

controller = FuzzyController(fuzzy_models, outputs=outputs)

mynode_ = None
pub_ = None
regions_ = {
    'right': 0,
    'fright': 0,
    'front1': 0,
    'front2': 0,
    'fleft': 0,
    'left': 0,
}
twstmsg_ = None

# main function attached to timer callback
def timer_callback():
    global pub_, twstmsg_
    if ( twstmsg_ != None ):
        pub_.publish(twstmsg_)


def clbk_laser(msg):
    global regions_, twstmsg_
    
    regions_ = {
        #LIDAR readings are anti-clockwise
        'front1':  find_nearest (msg.ranges[0:5]),
        'front2':  find_nearest (msg.ranges[355:360]),
        'right':  find_nearest(msg.ranges[265:275]),
        'fright': find_nearest (msg.ranges[310:320]),
        'fleft':  find_nearest (msg.ranges[40:50]),
        'left':   find_nearest (msg.ranges[85:95]),
        'bright': find_nearest(msg.ranges[210:220])
    }     
    twstmsg_= movement()

    
# Find nearest point
def find_nearest(list):
    f_list = filter(lambda item: item > 0.0, list)  # exclude zeros
    return min(min(f_list, default=0.9999), 0.9999)

#Basic movement method
def movement():
    #print("here")
    global regions_, mynode_, controller
    regions = regions_

    #create an object of twist class, used to express the linear and angular velocity of the turtlebot 
    msg = Twist()
    
    inputs = {
        'RFS': regions_['fright'], 
        'FS': regions_['front1'], 
        'LFS': regions_['fleft'], 
        'RBS':regions_['bright']
    }
    
    outputs = controller.tri_mf_control(inputs)
    
    msg.linear.x = outputs.get('Speed')
    msg.angular.z = -outputs.get('Direction')

    return msg
#used to stop the rosbot
def stop():
    global pub_
    msg = Twist()
    msg.angular.z = 0.0
    msg.linear.x = 0.0
    pub_.publish(msg)


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


    
