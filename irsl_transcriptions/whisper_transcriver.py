import base64
import io
import numpy as np
import torchaudio
import ffmpeg
import magic  
import whisper
from typing import Optional

class WhisperTranscriber:
    """
    Whisper-based audio transcriber that accepts base64 audio data without file saving.

    Attributes:
        model (whisper.model.Whisper): Preloaded Whisper model.
    """

    def __init__(self, model_size: str = "base") -> None:
        """
        Initializes the transcriber by loading the specified Whisper model.

        Args:
            model_size (str): Whisper model size, e.g. "tiny", "base", "small", "medium", "large".
        """
        self.model = whisper.load_model(model_size)

    def transcribe_from_base64(
        self, audio_base64_str: str, language: Optional[str] = None
    ) -> str:
        """
        Transcribes base64-encoded audio data.

        This method detects the audio format automatically, decodes and resamples as needed,
        then performs transcription with the loaded Whisper model.

        Args:
            audio_base64_str (str): Base64-encoded audio string.
            language (Optional[str]): Language code (e.g., "en", "ja"). If None, auto-detect.

        Returns:
            str: Transcribed text.

        Raises:
            ValueError: If audio format is unsupported.
        """
        audio_bytes = base64.b64decode(audio_base64_str)
        audio_file_like = io.BytesIO(audio_bytes)

        mime_type = magic.from_buffer(audio_bytes, mime=True)

        if mime_type in ["audio/wav", "audio/x-wav", "audio/flac", "audio/x-flac"]:
            waveform, sample_rate = torchaudio.load(audio_file_like)
            if sample_rate != 16000:
                resampler = torchaudio.transforms.Resample(orig_freq=sample_rate, new_freq=16000)
                waveform = resampler(waveform)
            audio_np = waveform.squeeze().numpy()

        elif mime_type in ["audio/mpeg", "audio/mp3", "audio/x-m4a", "audio/aac"]:
            process = (
                ffmpeg
                .input("pipe:0")
                .output("pipe:1", format="f32le", ac=1, ar="16000")
                .run_async(pipe_stdin=True, pipe_stdout=True, pipe_stderr=True)
            )
            out, err = process.communicate(input=audio_bytes)
            audio_np = np.frombuffer(out, np.float32)

        else:
            raise ValueError(f"Unsupported audio format: {mime_type}")

        result = self.model.transcribe(audio_np, language=language)
        return result["text"]
