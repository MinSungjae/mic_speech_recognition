#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
from pocketsphinx import pocketsphinx, Jsgf, FsgModel
import pyaudio

class OfflineSpeechRecognitionNode:
    def __init__(self):
        rospy.init_node('offline_speech_recognition_node', anonymous=True)
        self.pub = rospy.Publisher('/recognized_speech', String, queue_size=10)
        self.rate = rospy.Rate(1)  # 10hz

        # Configure PocketSphinx
        config = pocketsphinx.Decoder.default_config()
        config.set_string('-hmm', '/home/sungjae/ta_ws/pocketsphinx/model/en-us/en-us/')  # Path to the acoustic model
        config.set_string('-lm', '/home/sungjae/ta_ws/pocketsphinx/model/en-us/en-us.lm.bin')  # Path to the language model
        config.set_string('-dict', '/home/sungjae/ta_ws/pocketsphinx/model/en-us')  # Path to the dictionary
        self.decoder = pocketsphinx.Decoder(config)

        self.buffer_size = 48000

    def listen_and_recognize(self):
        p = pyaudio.PyAudio()
        stream = p.open(format=pyaudio.paInt16, channels=1, rate=8000, input=True, frames_per_buffer=self.buffer_size)
        
        stream.start_stream()
        in_speech_bf = False
        self.decoder.start_utt()
        while not rospy.is_shutdown():
            rospy.loginfo('Recording...')
            buf = stream.read(self.buffer_size)
            if buf:
                self.decoder.process_raw(buf, False, False)
                if self.decoder.get_in_speech() != in_speech_bf:
                    print('in')
                    in_speech_bf = self.decoder.get_in_speech()
                    if not in_speech_bf:
                        self.decoder.end_utt()
                        text = self.decoder.hyp().hypstr
                        rospy.loginfo("Recognized speech: %s", text)
                        self.pub.publish(text)
                        self.decoder.start_utt()
            else:
                break
        self.decoder.end_utt()

    def run(self):
        while not rospy.is_shutdown():
            self.listen_and_recognize()
            self.rate.sleep()

if __name__ == '__main__':
    try:
        node = OfflineSpeechRecognitionNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
