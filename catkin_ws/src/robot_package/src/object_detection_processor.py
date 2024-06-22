#!/usr/bin/env python

# Imports
import rospy
from darknet_ros_msgs.msg import BoundingBoxes, ObjectCount
from std_msgs.msg import String


class ObjectDetectionProcessor:
    def __init__(self):
        # Initialize the node
        rospy.init_node("object_detection_processor")

        # Subscribe to the bounding boxes topic
        self.bboxes_sub = rospy.Subscriber(
            "/darknet_ros/bounding_boxes", BoundingBoxes, self.bboxes_callback
        )

        # Subscribe to the speech recognition topic
        self.speech_sub = rospy.Subscriber(
            "/speech_recognition/audio", String, self.analyze_speech
        )

        # Start a publisher to publish the text
        self.tts_pub = rospy.Publisher("tts_text", String, queue_size=10)

        # Data about the objects
        self.data = {
            "person": {"price": 15, "location": "house"},
            "chair": {"price": 10, "location": "room"},
            "cell phone": {"price": 1000, "location": "room 2"},
        }

    # Bounding boxes callback function
    def bboxes_callback(self, msg):

        # For every bounding box received
        for bbox in msg.bounding_boxes:
            # Try-Except block for undefined items
            try:
                # Log the detected objects name
                rospy.loginfo(str(bbox.Class))

                # TODO: Fix Processing
                # price = self.data[bbox.Class]["price"]
                # self.publish_text(
                #     "{0} is priced at {1} dollars.".format(bbox.Class, price)
                # )
                self.publish_text("Item Detected")
            except KeyError:  # If an item is not found
                rospy.logwarn("Information for " + bbox.Class + " not found.")
                self.publish_text("Information for " + bbox.Class + " not found.")

    # Speech callback function
    def analyze_speech(self, msg):
        # Log and publish text
        rospy.loginfo("Received speech: " + msg.data)
        text = String()
        text.data = msg.data
        self.tts_pub.publish(text)


if __name__ == "__main__":
    try:
        processor = ObjectDetectionProcessor()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
