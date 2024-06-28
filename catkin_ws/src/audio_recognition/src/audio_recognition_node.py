#!/usr/bin/env python

# Imports
import numpy as np
import rospy
import sounddevice as sd
import soundfile as sf
import speech_recognition as sr
from std_msgs.msg import String


class AudioProcessor:
    def __init__(self):
        # Initialize the node
        rospy.init_node("audio_processor")
        
        # Start a publisher to publish the speech  recognizer
        self.pub = rospy.Publisher("/speech_recognition/audio", String, queue_size=10)

        # Sampling rate and duration
        self.sampling_rate = 16000
        self.duration = 3

    # Function to capture audio
    def capture_audio(self):
        rospy.loginfo("Capturing audio...")

        # Save recording
        recording = sd.rec(
            int(self.duration * self.sampling_rate),
            samplerate=self.sampling_rate,
            channels=1,
            dtype="int16",
        )
        sd.wait()

        # Convert int16 numpy array to bytes
        audio_bytes = recording.tobytes()

        # Use SpeechRecognition library to perform speech-to-text
        recognizer = sr.Recognizer()
        try:
            audio_data = sr.AudioData(
                audio_bytes, self.sampling_rate, 2
            )
            text = recognizer.recognize_google(audio_data)
            
            # Log and publish recognized speech
            rospy.loginfo("Recognized speech: " + text)
            self.pub.publish(text)
        except sr.UnknownValueError:
            rospy.logwarn("Speech recognition could not understand audio")
        except sr.RequestError as e:
            rospy.logerr("Speech recognition service error: " + e)


if __name__ == "__main__":
    try:
        audio_processor = AudioProcessor()
        rate = rospy.Rate(0.2)
        while not rospy.is_shutdown():
            audio_processor.capture_audio()
            rate.sleep()
    except rospy.ROSInterruptException:
        pass
