#!/usr/bin/env python

import rospy
from std_msgs.msg import String
import sounddevice as sd
import numpy as np
import soundfile as sf
import speech_recognition as sr

class AudioProcessor:
    def __init__(self):
        rospy.init_node('audio_processor')
        self.pub = rospy.Publisher('/speech_recognition/audio', String, queue_size=10)
        self.sampling_rate = 16000  # Define the sampling rate
        self.duration = 5  # Duration to record in seconds

    def capture_audio(self):
        rospy.loginfo("Capturing audio...")
        recording = sd.rec(int(self.duration * self.sampling_rate), samplerate=self.sampling_rate, channels=1, dtype='int16')
        sd.wait()  # Wait until the recording is finished
        
        # Convert int16 numpy array to bytes
        audio_bytes = recording.tobytes()
        
        # Use SpeechRecognition library to perform speech-to-text
        recognizer = sr.Recognizer()
        try:
            audio_data = sr.AudioData(audio_bytes, self.sampling_rate, 2)  # Provide 2 bytes per sample (required for int16)
            text = recognizer.recognize_google(audio_data)
            rospy.loginfo("Recognized speech: %s", text)
            self.pub.publish(text)
        except sr.UnknownValueError:
            rospy.logwarn("Speech recognition could not understand audio")
        except sr.RequestError as e:
            rospy.logerr("Speech recognition service error: %s", e)

if __name__ == '__main__':
    try:
        audio_processor = AudioProcessor()
        rate = rospy.Rate(0.2)  # Capture audio every 5 seconds
        while not rospy.is_shutdown():
            audio_processor.capture_audio()
            rate.sleep()
    except rospy.ROSInterruptException:
        pass

