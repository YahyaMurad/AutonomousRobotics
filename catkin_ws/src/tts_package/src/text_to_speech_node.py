#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from sound_play.libsoundplay import SoundClient

class TextToSpeechNode:
    def __init__(self):
        rospy.init_node('text_to_speech_node')
        self.sub = rospy.Subscriber('tts_text', String, self.tts_callback)
        self.soundhandle = SoundClient()
        rospy.spin()

    def tts_callback(self, msg):
        rospy.loginfo('Received text: %s' % msg.data)
        self.soundhandle.say(msg.data)

if __name__ == '__main__':
    try:
        TextToSpeechNode()
    except rospy.ROSInterruptException:
        pass

