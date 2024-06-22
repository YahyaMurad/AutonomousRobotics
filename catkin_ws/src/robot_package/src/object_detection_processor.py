#!/usr/bin/env python

import rospy
from darknet_ros_msgs.msg import BoundingBoxes, ObjectCount
from std_msgs.msg import String

class ObjectDetectionProcessor:
    def __init__(self):
        rospy.init_node('object_detection_processor')

        # Subscribe to the bounding boxes topic
        self.bboxes_sub = rospy.Subscriber('/darknet_ros/bounding_boxes', BoundingBoxes, self.bboxes_callback)

        # Subscribe to the object count topic
        self.count_sub = rospy.Subscriber('/darknet_ros/found_object', ObjectCount, self.count_callback)

        # Subscribe to the speech recognition topic
        self.speech_sub = rospy.Subscriber('/speech_recognition/audio', String, self.analyze_speech)

        # Publisher for tts_text
        self.tts_pub = rospy.Publisher('tts_text', String, queue_size=10)

        self.data = {
            "chair": {"price": 10, "location": "room"},
            "cell phone": {"price": 1000, "location": "room 2"}
        }

    def bboxes_callback(self, msg):
        """
        Callback function for the bounding boxes topic.
        This function will be called whenever a new bounding box message is received.
        """
        for bbox in msg.bounding_boxes:
            try:
                rospy.loginfo(str(bbox.Class))
                price = self.data[bbox.Class]["price"]
                self.publish_text("{0} is priced at {1} dollars.".format(bbox.Class, price))
            except KeyError:
                rospy.logwarn("Information for {0} not found.".format(bbox.Class))
                self.publish_text("Information for {0} not found.".format(bbox.Class))

    def count_callback(self, msg):
        """
        Callback function for the object count topic.
        This function will be called whenever a new object count message is received.
        """
        rospy.loginfo(str(msg.count))
        self.publish_text("Number of objects detected: {0}".format(msg.count))

    def analyze_speech(self, msg):
        """
        Callback function for the speech recognition topic.
        This function will be called whenever a new speech message is received.
        """
        rospy.loginfo("Received speech: " + msg.data)
        self.publish_text(msg.data)

    def publish_text(self, text):
        """
        Function to publish text to the tts_text topic.
        """
        msg = String()
        msg.data = text
        self.tts_pub.publish(msg)

if __name__ == '__main__':
    try:
        processor = ObjectDetectionProcessor()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
