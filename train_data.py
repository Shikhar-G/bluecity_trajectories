#!/home/shikhar/venv/bin/python 
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
  # Save the trajectories as a pickle file
  print(trajectories)
  with open("trajectory.pickle", "wb") as f:
    pickle.dump(trajectories, f)
  print("Trajectory data saved")

if __name__ == '__main__':
  rospy.init_node('bluecity_trajectories', anonymous=False)
  rospy.on_shutdown(save_data)

  # Trajectory dictionary mapping object id to a list of tuples
  # of the form (class, x, y, speed, angle)
  trajectories = {}

  # Loop while rospy is not shutdown. 
  while rospy.is_shutdown() == False:
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
            # BCTWSConnection.subscriptionOption.LOOP_CHANGE,
            # BCTWSConnection.subscriptionOption.PHASE_CHANGE,
            BCTWSConnection.subscriptionOption.FRAME])
    print("Stream opened")

    while rospy.is_shutdown() == False:
      data = stream.get_frame()
      # append to trajectory dictionary
      for obj in data.objects:
        if obj.id not in trajectories:
          trajectories[obj.id] = []
          # speed is an optional field
        if obj.speed is not None:
          trajectories[obj.id].append((obj.classType, obj.centerX, obj.centerY, obj.speed, obj.rotation))
        else:
          trajectories[obj.id].append((obj.classType, obj.centerX, obj.centerY, 0, obj.rotation))