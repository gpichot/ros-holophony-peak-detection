#!/usr/bin/env python

import rospy
from audiocorr.msg import audiocorr_result

import matplotlib.pyplot as plt

class Subscriber(object):
    def callback(self, msg):
        nb = len(msg.mic1)
        if not hasattr(self, 'figure'):
            self.figure, self.axes = plt.subplots(nb)
            self.figure.show()
        for i in range(0, nb):
            data = msg.data[0::nb]
            self.axes[i].plot(range(0, len(data)), data)

        self.figure.canvas.draw()
        self.figure.clf()
        
       
    

def listener():
    rospy.init_node('audiocorr_plot', anonymous=True)
    subscriber = Subscriber()

    rospy.Subscriber('audiocorr_result', audiocorr_result, subscriber.callback) 
    
    rospy.spin()


if __name__ == '__main__':
    listener()
