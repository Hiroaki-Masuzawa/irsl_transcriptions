from irsl_transcriptions.whisper_transcriber import WhisperTranscriber
import base64

# https://gist.github.com/juliensalinas/15789d241f28b1ce45f0c22e11ba894a
def audiopath_to_base64(audio_path):
    with open(audio_path, 'rb') as binary_file:
        binary_file_data = binary_file.read()
        base64_encoded_data = base64.b64encode(binary_file_data)
        base64_output = base64_encoded_data.decode('utf-8')
    return base64_output

if __name__ == '__main__':
    transcriber = WhisperTranscriber(model_size="large") # "base" 

    filename = "001-sibutomo.mp3"
    base64_audio = audiopath_to_base64("001-sibutomo.mp3")

    text = transcriber.transcribe_from_base64(base64_audio, language="ja")
    print("Transcription result:", text)

    text = transcriber.transcribe_audio(filename, language="ja")
    print("Transcription result:", text)

    