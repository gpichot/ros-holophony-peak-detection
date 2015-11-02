#!/usr/bin/env python

import rospy
from peak_detection.msg import SoundSources
import numpy as np
import matplotlib.patches as patches
import matplotlib.pyplot as plt


MICS_COUNT = 8
MICS_DISTANCE = 0.20 # m
MICS_START = 1 # m



class RoomPlot(object):
  def __init__(self, plt):
    self.plt = plt

  def callback(self, data):
    for intersection in data.intersections:
      rect = patches.Rectangle((intersection.x, intersection.y), 0.05, 0.05, facecolor="#aaaaaa")
      plt.gca().add_patch(rect)
    plt.draw()



		      

def listener():
        rospy.init_node('room_plot', anonymous=True)

        plt.figure(1)
        sap = RoomPlot(plt)

        rospy.Subscriber("sound_sources", SoundSources, sap.callback)

        for i in range(0, MICS_COUNT):
            y = MICS_START + i * MICS_DISTANCE
            rect = patches.Rectangle((0, y), 0.05, 0.05, facecolor="#aaaaaa")
            plt.gca().add_patch(rect)
            plt.text(-0.25, y, i)

        plt.xticks(np.arange(-0.5, 8, 1.0))
        plt.yticks(np.arange(0, MICS_START + MICS_COUNT * MICS_DISTANCE + MICS_START * 1.5, 1.0))
        plt.show()

        rospy.spin()
if __name__ == '__main__':
  listener()
