#!/usr/bin/env python3
import re

import rospy
from std_msgs.msg import String

from irsl_transcriptions.whisper_transcriber import WhisperTranscriber


class IRSLTranscriptionROS:
    """
    ROS node for real-time audio transcription using Whisper.

    This class subscribes to a topic containing base64-encoded audio data,
    decodes and transcribes it using the Whisper model, and publishes the
    resulting text to another topic.

    Attributes:
        transcriber (WhisperTranscriber): Instance of the Whisper transcriber.
        sub (rospy.Subscriber): Subscriber for base64 audio input.
        pub (rospy.Publisher): Publisher for transcribed text output.
    """

    def __init__(self):
        """
        Initialize the ROS transcription node.

        Parameters retrieved from the ROS parameter server:
            ~model_name (str): Whisper model size (default: 'base').
            ~language (str): Optional language code (e.g., 'en', 'ja').
        """
        rospy.init_node('transcription', anonymous=True)

        model_name = rospy.get_param('~model_name', 'base')
        self.language = rospy.get_param('~language', None)

        rospy.loginfo(f"Loading Whisper model: {model_name}")
        self.transcriber = WhisperTranscriber(model_size=model_name)

        self.sub = rospy.Subscriber("audio_in", String, self.callback)
        self.pub = rospy.Publisher("text_out", String, queue_size=10)

    def callback(self, msg):
        """
        Callback function triggered upon receiving audio input.

        Extracts and decodes base64-encoded audio data, performs transcription
        using Whisper, and publishes the resulting text.

        Args:
            msg (std_msgs.msg.String): Base64-encoded audio message.
        """
        try:
            data = msg.data

            # Handle Data URL scheme (e.g. "data:audio/wav;base64,...")
            if data.startswith("data:"):
                match = re.match(r"data:audio/\w+;base64,(.*)", data)
                if match:
                    data = match.group(1)
                else:
                    rospy.logwarn("Invalid data URL format. Skipping message.")
                    return

            text = self.transcriber.transcribe_from_base64(data, language=self.language)

            rospy.loginfo(f"Transcription: {text}")
            self.pub.publish(String(data=text))

        except Exception as e:
            rospy.logerr(f"Transcription error: {e}")


if __name__ == '__main__':
    # Start the transcription node and keep it alive.
    transcription = IRSLTranscriptionROS()
    rospy.spin()
