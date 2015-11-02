#!/usr/bin/env python

import rospy
from peak_detection.msg import SoundSources


def callback(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
    
def listener():
    rospy.init_node('room_plot', anonymous=True)

    rospy.Subscriber("sound_sources", SoundSources, callback)

    rospy.spin()

if __name__ == '__main__':
    listener()
