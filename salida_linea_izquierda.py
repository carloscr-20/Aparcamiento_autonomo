from cmath import cos, pi
import numpy as np

import math
import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
from tf.transformations import euler_from_quaternion, quaternion_from_euler

pos1 = 0
pos2 = 0
objetivo = 0
roll = pitch = yaw = 0.0
target = 45
target1 = 0
target2 = 0
target_const = math.pi/4
control = 0
control0 = 0

class Safety:

  

  def __init__(self):
    LIDAR_TOPIC = "/scan"
    DRIVE_TOPIC = "/vesc/high_level/ackermann_cmd_mux/input/nav_0"
    ODOM_TOPIC = "/vesc/odom"
   


    ##################################################
    # TODO >>>
    self.drive_msg=AckermannDriveStamped()

    ##################################################

    ##################################################
    # <<< TODO
    ##################################################

    # Initialize a publisher for drive messages
    self.drive_pub = rospy.Publisher(
        DRIVE_TOPIC,
        AckermannDriveStamped,
        queue_size=1)

    # Subscribe to the laser scan data
    rospy.Subscriber(
        LIDAR_TOPIC,
        LaserScan,
        self.callback)

    # Suscribe to Odometry
    rospy.Subscriber(
        ODOM_TOPIC,
        Odometry,
        self.position)


  def callback(self, msg):
    ##################################################
    # TODO >>>
    # Make the robot move, but don't let it get too
    # close to obstacles!
    ##################################################

    if np.min(msg.ranges) > 0.2:
      self.drive_msg.drive.speed = 0.3
    else:
      self.drive_msg.drive.speed = 0
    # self.drive_pub.publish(self.drive_msg)
    
    ##################################################
    # <<< TODO
    ##################################################


  def position(self, msg):
    ##################################################
    # TODO >>>
    ##################################################
    global pos1, pos2, objetivo, roll, pitch, yaw, target1, target2, target_const, control, control0
    position_y = msg.pose.pose.position.y
    orientation_q = msg.pose.pose.orientation
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    (roll, pitch, yaw) = euler_from_quaternion (orientation_list)

    if control == 0:
      target1 = abs(yaw) - target_const
      target2 = abs(yaw)
      control = 1

    elif abs(yaw) > target1 and pos1 == 0:
        self.drive_msg.drive.steering_angle = -0.7
        self.drive_msg.drive.speed = 0.3
  
    elif abs(yaw) <= target1 and pos1 == 0:
      pos1 = 1
      if yaw > 0:
        objetivo = position_y + 0.25
      elif yaw <= 0:
        objetivo = position_y - 0.25
      
      self.drive_msg.drive.steering_angle = 0
      self.drive_msg.drive.speed = 0.3
     
    elif yaw < 0 and pos1 == 1 and pos2 == 0: 
      if objetivo > position_y and pos1 == 1 and pos2 == 0:
        self.drive_msg.drive.steering_angle = 0
        self.drive_msg.drive.speed = 0.3
     
      elif objetivo <= position_y and pos1 == 1 and pos2 == 0:
        pos2 = 1

    elif yaw >= 0 and pos1 == 1 and pos2 == 0: 
      if objetivo < position_y and pos1 == 1 and pos2 == 0:
        self.drive_msg.drive.steering_angle = 0
        self.drive_msg.drive.speed = 0.3
     
      elif objetivo >= position_y and pos1 == 1 and pos2 == 0:
        pos2 = 1

    elif abs(yaw) < target2 and pos1 == 1 and pos2 == 1:
        self.drive_msg.drive.steering_angle = 0.7
        self.drive_msg.drive.speed = 0.3
    
    elif abs(yaw) >= target2 or yaw < 0:
      if pos1 == 1 and pos2 == 1:
        self.drive_msg.drive.steering_angle = 0
        self.drive_msg.drive.speed = 0
      
    self.drive_pub.publish(self.drive_msg)

    ##################################################
    # <<< TODO


if __name__ == "__main__":
  rospy.init_node("Safety")
  mover = Safety()
  rospy.spin()
