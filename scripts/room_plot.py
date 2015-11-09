#!/usr/bin/env python

import rospy
from peak_detection.msg import SoundSources
import numpy as np
import matplotlib.patches as patches
from matplotlib import animation
import matplotlib.pyplot as plt
import math


MICS_COUNT = 8
MICS_DISTANCE = 0.20 # m



class RoomPlot(object):
  def __init__(self, plt):
    self.plt = plt
    self.draw_mics()

  def position_mic(self, i):
      return (MICS_COUNT - 1 - i) * MICS_DISTANCE

  def draw_mics(self):
        for i in range(0, MICS_COUNT):
            x = self.position_mic(i)
            rect = patches.Rectangle((x, 0), 0.05, 0.05, facecolor="#aaaaaa")
            self.plt.gca().add_patch(rect)
            self.plt.text(x, -0.25, i)

  def callback(self, data):
    self.plt.clf()
    self.plt.ion()
    self.plt.autoscale(enable=False, tight=False)
#   self.plt.axis("equal")
    self.plt.xlim(-2, 4)
    self.plt.ylim(-1, MICS_COUNT * MICS_DISTANCE)
    self.draw_mics()
    for intersection in data.intersections:
        rect = patches.Rectangle((intersection.x, intersection.y), 0.05, 0.05, facecolor="#aaaaaa")
        self.plt.gca().add_patch(rect)
        print "(x: %f, y: %f)" % (intersection.x, intersection.y)

    for direction in data.directions:
        if math.tan(direction.theta) != 0:
            x = 0
            # C'etait peut etre mieux avant...            
            if direction.theta < 0:
                y = (direction.x) / math.tan(- direction.theta)
            else:
                y = - (direction.x) * math.tan(math.pi / 2 - direction.theta)

            if y < 0:
                x = 2 * (direction.x)
                y = -y
        else:
            x = direction.x
            y = 3
        self.plt.plot([direction.x, x], [0, y], scalex=False, scaley=False);
        print "(x: %f, theta: %f)" % (direction.x, direction.theta * 180 / 3.1416)

    self.plt.draw()

def listener():
        rospy.init_node('room_plot', anonymous=True)

        plt.figure(1)
        sap = RoomPlot(plt)

        rospy.Subscriber("sound_sources", SoundSources, sap.callback)

        plt.show()

        rospy.spin()
if __name__ == '__main__':
  listener()
