#!/usr/bin/env python

# Imports
import rospy
from sound_play.libsoundplay import SoundClient
from std_msgs.msg import String


class TextToSpeechNode:
    def __init__(self):
	print("INIT")        
# Initialize the node
        rospy.init_node("text_to_speech_node")

        # Subscribe to the text topic
        self.sub = rospy.Subscriber("tts_text", String, self.tts_callback)
	print("SUBSCRIBER")
        # Initialize the SoundClient
        self.soundhandle = SoundClient()

        # Call rospy.spin()
        rospy.spin()

    # Function to log and speak the message
    def tts_callback(self, msg):
	print("CALLBACK")
        # Logging the message received
        rospy.loginfo("Received text: " + msg.data)

        # Speaking the message received
        self.soundhandle.say(msg.data)


if __name__ == "__main__":
    try:
        TextToSpeechNode()
    except rospy.ROSInterruptException:
        pass

