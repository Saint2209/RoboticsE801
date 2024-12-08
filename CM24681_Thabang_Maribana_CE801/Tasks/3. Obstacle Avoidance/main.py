
import rclpy
import math
import numpy as np
import os
import sys

movement_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..', 'Movement')
sys.path.append(movement_path)

from FuzzyLogicSystem import Fuzzer

from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf2_ros import TransformRegistration
from rclpy.qos import QoSProfile, ReliabilityPolicy

current_dir = os.path.dirname(os.path.abspath(__file__))

# Path to the JSON and CSV files
json_path = os.path.join(current_dir, 'object_avoidance.json')
csv_path = os.path.join(current_dir, 'object_avoidance.csv')


object_avoidance = Fuzzer( json_path = json_path,
    csv_path=csv_path,
    inputs=['RFS', 'FS','LFS'],
    outputs=['Speed','Direction']
)

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
    global regions_, mynode_, object_avoidance
    regions = regions_

    front_region = np.min([regions_['front1']])

    print('RF: ', front_region)
    print('LFS: ', regions_['fleft'])
    print('RFS: ', regions_['fright'])
    #create an object of twist class, used to express the linear and angular velocity of the turtlebot 
    msg = Twist()
    
    print('here')
    object_avoidance.set_input('RFS', regions_['fright'])
    object_avoidance.set_input('FS',front_region)
    object_avoidance.set_input('LFS', regions_['fleft'])

    print('here')
    object_avoidance()
    output = object_avoidance.get_outputs() 

    print('here')
    msg.linear.x = output.get('Speed')
    msg.angular.z = output.get('Direction')

    print(f'Angular z: {msg.angular.z}')

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


    
