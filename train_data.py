#!/usr/bin/env python3
from realtime_subscriber.Realtime_subscriber_api import BCTWSConnection
import threading
import sys
import math
import rospy, roslib
import cv2
import numpy as np
import os
import pickle

roslib.load_manifest('amrl_msgs')
import rospkg

from amrl_msgs.msg import VisualizationMsg
from amrl_msgs.msg import ColoredLine2D
from amrl_msgs.msg import Point2D

stream = None
arr = []

def save_data():
  # Save the array as a pickle file
  print("save data")
  with open("array.pickle", "wb") as f:
      pickle.dump(arr, f)

if __name__ == '__main__':
  rospy.init_node('bluecity_example', anonymous=False)
  rospy.on_shutdown(save_data)
  if not os.path.exists("images"):
    os.makedirs("images")
  count = 0
  # Check to see if a file named ".credentials" exists in the current directory,
  # and if so, use it to log in
  try:
      with open(".credentials") as f:
          username = f.readline().strip()
          password = f.readline().strip()
  except IOError:
      # If the file doesn't exist, use the command line arguments
      # Accept username as the first argument and password as the second argument
      if len(sys.argv) != 3:
          print("Usage: python example.py <username> <password>")
          sys.exit(1)
      username = sys.argv[1]
      password = sys.argv[2]

  print("Opening Blucity stream...")
  stream = BCTWSConnection(
      "BCT_3D_4G_0206001",
      username,
      password,
      singleton=False,
      subscriptions = [
          BCTWSConnection.subscriptionOption.LOOP_CHANGE,
          BCTWSConnection.subscriptionOption.PHASE_CHANGE,
          BCTWSConnection.subscriptionOption.FRAME])
  print("Stream opened")

  sensorLoc = Point2D()
  sensorLoc.x = 86
  sensorLoc.y = -120
  sensorAngle = math.radians(5)
 
  print("here ", rospy.is_shutdown())
  while rospy.is_shutdown() == False:
    # print("Getting frame...")
    data = stream.get_frame()
    print("length", len(data.objects))
    # print("Frame: ")
    # print(len(data.objects))
    if count %5 ==0:
      print("appending")
      arr.append(data)
        
    count+=1
    #   msg.lines.extend(DrawBox(obj.centerX, obj.centerY, obj.length, obj.width, obj.rotation, color))
    # pub.publish(msg)