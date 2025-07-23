#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
import base64

# https://gist.github.com/juliensalinas/15789d241f28b1ce45f0c22e11ba894a
def audiopath_to_base64(audio_path):
    with open(audio_path, 'rb') as binary_file:
        binary_file_data = binary_file.read()
        base64_encoded_data = base64.b64encode(binary_file_data)
        base64_output = base64_encoded_data.decode('utf-8')
    return base64_output

def callback(msg):
    print(msg.data)

if __name__ == '__main__':
    base64_audio = audiopath_to_base64("001-sibutomo.mp3")
    # print(base64_audio[:10])
    # print( base64.b64decode( base64_audio )[:10])
    # exit()
    rospy.init_node("transcription_test")
    pub = rospy.Publisher('audio_in', String, queue_size=10)
    sub = rospy.Subscriber("text_out", String, callback)

    while pub.get_num_connections() == 0:
        rospy.sleep(0.1)

    pub.publish(String(data=base64_audio))

    rospy.sleep(10)

