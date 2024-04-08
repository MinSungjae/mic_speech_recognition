#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
import speech_recognition as sr

class SpeechRecognitionNode:
    def __init__(self):
        rospy.init_node('speech_recognition_node', anonymous=True)
        self.pub = rospy.Publisher('/recognized_speech', String, queue_size=10)
        self.rate = rospy.Rate(10)  # 10hz
        self.recognizer = sr.Recognizer()

    def listen_and_recognize(self):
        with sr.Microphone() as source:
            print("Listening...")
            audio = self.recognizer.listen(source)

            try:
                text = self.recognizer.recognize_google(audio)
                rospy.loginfo("Recognized speech: %s", text)
                self.pub.publish(text)
            except sr.UnknownValueError:
                rospy.logwarn("Could not understand audio")
            except sr.RequestError as e:
                rospy.logerr("Could not request results; {0}".format(e))

    def run(self):
        while not rospy.is_shutdown():
            self.listen_and_recognize()
            self.rate.sleep()

if __name__ == '__main__':
    try:
        node = SpeechRecognitionNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
