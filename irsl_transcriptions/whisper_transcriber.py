import base64
import io
import numpy as np
import torch
import torchaudio
import ffmpeg
import magic
import whisper
from typing import Optional, Union


class WhisperTranscriber:
    """
    Transcribes base64-encoded or in-memory audio using OpenAI's Whisper model.

    Attributes:
        model (whisper.model.Whisper): Loaded Whisper model instance.
    """

    def __init__(self, model_size: str = "base") -> None:
        """
        Initialize the Whisper model with the specified size.

        Args:
            model_size (str): Size of the Whisper model to load.
                Options: "tiny", "base", "small", "medium", "large".
        """
        device = "cuda" if torch.cuda.is_available() else "cpu"
        self.model = whisper.load_model(model_size).to(device)

    def transcribe_from_base64(
        self, audio_base64_str: str, language: Optional[str] = None
    ) -> str:
        """
        Decode and transcribe base64-encoded audio data.

        Automatically detects audio format and handles necessary resampling
        before passing it to the Whisper model.

        Args:
            audio_base64_str (str): Base64-encoded audio string.
            language (Optional[str]): Language code (e.g., "en", "ja").
                If None, language will be auto-detected.

        Returns:
            str: Transcribed text.

        Raises:
            ValueError: If the audio format is not supported.
        """
        audio_bytes = base64.b64decode(audio_base64_str)
        audio_file_like = io.BytesIO(audio_bytes)

        mime_type = magic.from_buffer(audio_bytes, mime=True)

        if mime_type in ["audio/wav", "audio/x-wav", "audio/flac", "audio/x-flac"]:
            # Load waveform using torchaudio
            waveform, sample_rate = torchaudio.load(audio_file_like)
            if sample_rate != 16000:
                # Resample to 16 kHz
                resampler = torchaudio.transforms.Resample(
                    orig_freq=sample_rate, new_freq=16000
                )
                waveform = resampler(waveform)
            audio_np = waveform.squeeze(0).numpy()

        elif mime_type in ["audio/mpeg", "audio/mp3", "audio/x-m4a", "audio/aac"]:
            # Use ffmpeg to decode and resample
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

        return self.transcribe_audio(audio_np, language)

    def transcribe_audio(
        self, audio_input: Union[str, np.ndarray, io.IOBase],
        language: Optional[str] = None
    ) -> str:
        """
        Transcribe audio from a file path, numpy array, or file-like object.

        Args:
            audio_input (Union[str, np.ndarray, io.IOBase]): Audio data or file path.
            language (Optional[str]): Language code. If None, auto-detected.

        Returns:
            str: Transcribed text.
        """
        result = self.model.transcribe(audio_input, language=language)
        return result["text"]
