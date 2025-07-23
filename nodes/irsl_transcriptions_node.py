#!/usr/bin/env python3

import rospy
from std_msgs.msg import String

from irsl_transcriptions.whisper_transcriver import WhisperTranscriber

class IRSLTranscriptionROS:
    """ROS node for real-time audio transcription using Whisper.

    This class initializes a ROS node that subscribes to a topic containing
    base64-encoded audio data, performs speech transcription using the
    Whisper model, and publishes the resulting text to a topic.

    Attributes:
        transcriber (WhisperTranscriber): An instance of the Whisper transcriber.
        sub (rospy.Subscriber): Subscriber for input audio data in base64 format.
        pub (rospy.Publisher): Publisher for output transcribed text.
    """

    def __init__(self):
        """
        Initializes the transcription ROS node.

        The Whisper model name is retrieved from the ROS parameter server.
        Parameter:
            ~model_name (str): Whisper model to use (default: 'base').
        """
        # Initialize the ROS node.
        rospy.init_node('transcription', anonymous=True)

        # Get Whisper model name from ROS parameter
        model_name = rospy.get_param('~model_name', 'base')

        rospy.loginfo(f"Loading Whisper model: {model_name}")
        self.transcriber = WhisperTranscriber(model_size=model_name)

        # Subscribe to incoming base64-encoded audio messages.
        self.sub = rospy.Subscriber("audio_in", String, self.callback)

        # Publisher to publish the transcribed text.
        self.pub = rospy.Publisher('text_out', String, queue_size=10)

    def callback(self, msg):
        """Callback function for handling incoming audio data.

        This method receives base64-encoded audio from the 'chatter' topic,
        transcribes it using Whisper, and publishes the transcribed text.

        Args:
            msg (std_msgs.msg.String): The message containing base64-encoded audio.
        """
        try:
            # Perform transcription from base64 audio.
            data = msg.data
            keyword = ',' # data url の分離　exp "data:audio/wav;base64,"
            if data.find(keyword) != -1:
                data = data[data.find(keyword)+len(keyword):]
                
            text = self.transcriber.transcribe_from_base64(data, language=None)
            rospy.loginfo(f"Transcription : {text}")
            # Publish the transcribed text.
            self.pub.publish(String(data=text))
        except Exception as e:
            rospy.logerr(f"Transcription error: {e}")

if __name__ == '__main__':
    # Create the transcription node with the specified Whisper model.
    transcription = IRSLTranscriptionROS()

    # Keep the node running and responsive to incoming messages.
    rospy.spin()
