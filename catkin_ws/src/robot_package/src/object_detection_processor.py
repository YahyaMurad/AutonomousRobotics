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
            "person": {"price": 15, "location": "student area", "description": "A homosapien"},
            "chair": {"price": 10, "location": "room", "description": "A piece of furniture"},
            "cell phone": {"price": 1000, "location": "room 2", "description": "A mobile device"},
            "bottle": {"price": 5, "location": "room 3", "description": "A container to store liquids"},
            "cup": {"price": 3, "location": "room 4", "description": "A container to drink liquids"},
            "book": {"price": 7, "location": "room 5", "description": "A collection of pages"},
            "laptop": {"price": 1000, "location": "room 6", "description": "A portable computer"},
            "keyboard": {"price": 50, "location": "room 7", "description": "A device to input text"},
            "mouse": {"price": 30, "location": "room 8", "description": "A device to control the cursor"},
            "monitor": {"price": 200, "location": "room 9", "description": "A display device"},
            "table": {"price": 50, "location": "room 10", "description": "A piece of furniture"},
            "tvmonitor": {"price": 500, "location": "room 11", "description": "A television display"},
            "remote": {"price": 10, "location": "room 12", "description": "A device to control other devices"},
            "toaster": {"price": 20, "location": "room 13", "description": "A device to toast bread"},
            "refrigerator": {"price": 500, "location": "room 14", "description": "A device to store food"},
            "oven": {"price": 300, "location": "room 15", "description": "A device"},
            "sink": {"price": 50, "location": "room 16", "description": "A device to wash hands"},
            "microwave": {"price": 100, "location": "room 17", "description": "A device to heat food"},
            "diningtable": {"price": 100, "location": "room 18", "description": "A table to eat food"},
            "bowl": {"price": 5, "location": "room 19", "description": "A container to eat food"},

        }
         # Initialize variables to store bounding box class and speech text
        self.detected_class = None
        self.speech_text = None

    # Bounding boxes callback function
    def bboxes_callback(self, msg):

        # For every bounding box received
        for bbox in msg.bounding_boxes:
                 # Store the detected class
            self.detected_class = bbox.Class
                

    # Speech callback function
    def analyze_speech(self, msg):
        # Log and publish text
        rospy.loginfo("Received speech: " + msg.data)
        text = String()
        text.data = msg.data

        # make sure that the text is correct
        self.speech_text = str(text)
        self.process_answer()

        
    # process the detected object and speech text
    def process_answer(self):
        self.speech_text = self.speech_text.strip()
        
        # check if the detected object is in the data
        if self.detected_class in self.data:
           
            if 'where' in self.speech_text or 'location' in self.speech_text or 'where is' in self.speech_text :
                location = self.data[self.detected_class]["location"]
                self.publish_text(
                    "{0} is located in the {1}.".format(self.detected_class, location)
                )
            elif 'what' in self.speech_text or 'description' in self.speech_text or 'what is' in self.speech_text:
                description = self.data[self.detected_class]["description"]
                self.publish_text(
                    "{0} is {1}.".format(self.detected_class, description)
                )
            elif 'price' in self.speech_text or "cost" in self.speech_text or 'how much' in self.speech_text or 'much' in self.speech_text :
                price = self.data[self.detected_class]["price"]
                self.publish_text(
                    "{0} is priced at {1} Ringgit.".format(self.detected_class, price)
                )
            else:
                self.publish_text("Soorry I did not understand the question")
        else:
            self.publish_text("I do not know what that is object is")
            

    def publish_text(self, text):
	msg = String()
	msg.data = text
	self.tts_pub.publish(msg)  
        


if __name__ == "__main__":
    try:
        processor = ObjectDetectionProcessor()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

