#!/usr/bin/env python2
#include "utils.h"
import os

from cmath import cos, pi
from turtle import delay
import numpy as np
from dec_linea_der import Target_X
import prueba_bat_der
import math
import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
import re
from tf.transformations import euler_from_quaternion, quaternion_from_euler

array_deteccion = 0
control = 0
control2 = 0
detection = 0
colocado = 0
aparcado = 0
determin_pos_y = 0
Target_Y = 0
pos1 = 0
pos2 = 0
objetivo = 0
roll = pitch = yaw = 0.0
target1 = 0
target2 = 0
target_const = math.pi/2 - 0.13
control3 = 0
aparcado = 0
seguridad = 0
yaw2 = 0

def deteccion (self, msg):
    detection = 0
    control = 0
    array_deteccion= [msg.ranges[930:960]]
    self.drive_msg.drive.speed = 0.4

    if np.min(array_deteccion) > 1 and control==0:
      detection = 1
      control = 1
      print ("Aparcamiento detectado")
    
    return detection

def colocacion (self, msg):
  global determin_pos_y, Target_Y
  colocado = 0
  position_y = msg.pose.pose.position.y

  if determin_pos_y == 0:
      Target_Y = position_y - 0.72
      determin_pos_y = 1
      self.drive_msg.drive.speed = 0.4

    
  if determin_pos_y == 1 and colocado == 0:
    if position_y > Target_Y:
        self.drive_msg.drive.speed = 0.4

    elif position_y <= Target_Y:
        colocado = 1
        self.drive_msg.drive.speed = 0.0
        print ("Colocado")

  elif determin_pos_y == 1 and control == 1 and aparcado == 0:
      self.drive_msg.drive.speed = 0.0
      print ("Colocado")

  return colocado

def maniobra (self, msg):
    global pos1, pos2, objetivo, roll, pitch, yaw, target1, target2, target_const, control, control3, yaw2
    aparcado = 0
    position_x = msg.pose.pose.position.x
    position_y = msg.pose.pose.position.y
    orientation_q = msg.pose.pose.orientation
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    (roll, pitch, yaw) = euler_from_quaternion (orientation_list)
    yaw = round(yaw,2)
    

    if control == 0:
      target1 = yaw - target_const
      control = 1

    if yaw > target1 and pos1 == 0 and pos2 == 0: 
      self.drive_msg.drive.steering_angle = 0.7
      self.drive_msg.drive.speed = -0.3
 
    if yaw <= target1 or yaw <= -3 or yaw2 == 4:
      if pos1 == 0 and pos2 == 0:
        yaw2 = 4
        pos1 = 1
        self.drive_msg.drive.steering_angle = 0.0
        objetivo = position_x + 0.40
      elif pos1 == 1 and pos2 == 0:
        if position_x < objetivo:
          self.drive_msg.drive.speed = -0.3
        elif position_x >= objetivo:
          self.drive_msg.drive.speed = 0.0
          pos2 == 1
          aparcado = 1
          print ("Aparcado")

      else:
        self.drive_msg.drive.steering_angle = 0.0
        self.drive_msg.drive.speed = 0.0
        aparcado = 1
        print ("Aparcado")

    return aparcado



class Parking_Linea_der:


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
    global array_deteccion, seguridad, detection, control

    if detection == 0:
      detection = deteccion(self, msg)
      self.drive_pub.publish(self.drive_msg)

    if np.min(msg.ranges) < 0.2:
      seguridad = 1

    
    ##################################################
    # <<< TODO
    ##################################################


  def position(self, msg):
    ##################################################
    # TODO >>>
    ##################################################
    global array_deteccion, control, control2, detection, colocado, aparcado, determin_pos_y
    
    

    if np.min(array_deteccion) > 1 and control==0:
      detection = 1
      control = 1
      print ("Aparcamiento detectado")

    

    if control2 == 0:
      colocado = 0
      aparcado = 0
      control2 = 1

    elif detection == 1 and colocado == 0:
      colocado = colocacion(self, msg)

    elif detection == 1 and colocado == 1 and aparcado == 0:
      aparcado = maniobra(self, msg)

    else:
      self.drive_msg.drive.speed = 0.0
    
    if seguridad == 1:
      self.drive_msg.drive.speed = 0.0
      if detection == 0:
        print ("Parada de seguridad")
      detection = 1
      colocado = 1
      aparcado = 1

    self.drive_pub.publish(self.drive_msg)

    ##################################################
    # <<< TODO


if __name__ == "__main__":
  rospy.init_node("Parking_Linea_der")
  mover = Parking_Linea_der()
  rospy.spin()