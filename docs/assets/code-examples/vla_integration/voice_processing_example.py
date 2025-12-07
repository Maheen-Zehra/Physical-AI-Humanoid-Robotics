#!/usr/bin/env python3
"""
Voice Processing Example for Humanoid Robot

This example demonstrates the voice processing pipeline using OpenAI Whisper
for speech-to-text conversion in the humanoid robot system.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import AudioData
import pyaudio
import wave
import threading
import queue
import time
import openai
import os
from typing import Optional


class VoiceProcessingExample(Node):
    """
    Example implementation of voice processing for humanoid robot.
    """

    def __init__(self):
        super().__init__('voice_processing_example')

        # Parameters
        self.declare_parameter('whisper_model', 'whisper-1')
        self.declare_parameter('audio_chunk_size', 1024)
        self.declare_parameter('sample_rate', 16000)
        self.declare_parameter('channels', 1)
        self.declare_parameter('record_seconds', 5)

        self.whisper_model = self.get_parameter('whisper_model').value
        self.chunk_size = self.get_parameter('audio_chunk_size').value
        self.sample_rate = self.get_parameter('sample_rate').value
        self.channels = self.get_parameter('channels').value
        self.record_seconds = self.get_parameter('record_seconds').value

        # OpenAI API key from environment
        self.openai_api_key = os.getenv('OPENAI_API_KEY')
        if not self.openai_api_key:
            self.get_logger().warn('OPENAI_API_KEY environment variable not set')

        # Audio processing setup
        self.audio_queue = queue.Queue()
        self.recording = False
        self.audio = None
        self.stream = None

        # Publishers
        self.transcript_pub = self.create_publisher(String, '/voice/transcript', 10)
        self.command_pub = self.create_publisher(String, '/voice/command', 10)

        # Timer for audio recording
        self.recording_timer = self.create_timer(0.1, self.recording_callback)

        self.get_logger().info('Voice Processing Example initialized')

    def recording_callback(self):
        """
        Handle audio recording if enabled.
        """
        if not self.recording:
            self.start_recording()

    def start_recording(self):
        """
        Start audio recording using PyAudio.
        """
        try:
            self.recording = True
            self.audio = pyaudio.PyAudio()

            # Open audio stream
            self.stream = self.audio.open(
                format=pyaudio.paInt16,
                channels=self.channels,
                rate=self.sample_rate,
                input=True,
                frames_per_buffer=self.chunk_size
            )

            self.get_logger().info('Started audio recording')

            # Record audio in a separate thread
            record_thread = threading.Thread(target=self.record_audio)
            record_thread.daemon = True
            record_thread.start()

        except Exception as e:
            self.get_logger().error(f'Error starting audio recording: {str(e)}')
            self.recording = False

    def record_audio(self):
        """
        Record audio in a separate thread.
        """
        try:
            frames = []

            for _ in range(0, int(self.sample_rate / self.chunk_size * self.record_seconds)):
                if not self.recording:
                    break
                data = self.stream.read(self.chunk_size)
                frames.append(data)

            # Stop recording
            self.stop_recording()

            # Process recorded audio
            self.process_audio(frames)

        except Exception as e:
            self.get_logger().error(f'Error recording audio: {str(e)}')
            self.recording = False

    def stop_recording(self):
        """
        Stop audio recording.
        """
        try:
            if self.stream:
                self.stream.stop_stream()
                self.stream.close()
            if self.audio:
                self.audio.terminate()
            self.recording = False
            self.get_logger().info('Stopped audio recording')
        except Exception as e:
            self.get_logger().error(f'Error stopping audio recording: {str(e)}')

    def process_audio(self, frames: list):
        """
        Process recorded audio frames and convert to text using Whisper.
        """
        try:
            # Create WAV file from frames
            import io
            wav_buffer = io.BytesIO()

            with wave.open(wav_buffer, 'wb') as wf:
                wf.setnchannels(self.channels)
                wf.setsampwidth(pyaudio.get_sample_size(pyaudio.paInt16))
                wf.setframerate(self.sample_rate)
                wf.writeframes(b''.join(frames))

            # Convert to bytes for OpenAI API
            wav_data = wav_buffer.getvalue()

            # Save temporary file for OpenAI API (required format)
            temp_filename = '/tmp/temp_audio.wav'
            with open(temp_filename, 'wb') as f:
                f.write(wav_data)

            # Process with OpenAI Whisper
            transcript = self.transcribe_audio(temp_filename)

            if transcript:
                self.publish_transcript(transcript)

            # Clean up temp file
            if os.path.exists(temp_filename):
                os.remove(temp_filename)

        except Exception as e:
            self.get_logger().error(f'Error processing audio: {str(e)}')

    def transcribe_audio(self, audio_file_path: str) -> Optional[str]:
        """
        Transcribe audio using OpenAI Whisper API.
        """
        if not self.openai_api_key:
            self.get_logger().error('OpenAI API key not set')
            return None

        try:
            # Set OpenAI API key
            openai.api_key = self.openai_api_key

            # Transcribe audio
            with open(audio_file_path, 'rb') as audio_file:
                transcript_response = openai.Audio.transcribe(
                    model=self.whisper_model,
                    file=audio_file
                )

            transcript = transcript_response.get('text', '').strip()
            self.get_logger().info(f'Transcribed: {transcript}')

            return transcript

        except Exception as e:
            self.get_logger().error(f'Error transcribing audio: {str(e)}')
            return None

    def publish_transcript(self, transcript: str):
        """
        Publish the transcribed text to ROS topics.
        """
        try:
            # Publish to transcript topic
            transcript_msg = String()
            transcript_msg.data = transcript
            self.transcript_pub.publish(transcript_msg)

            # Publish to command topic for processing
            command_msg = String()
            command_msg.data = transcript
            self.command_pub.publish(command_msg)

            self.get_logger().info(f'Published transcript: {transcript}')

        except Exception as e:
            self.get_logger().error(f'Error publishing transcript: {str(e)}')


def main(args=None):
    rclpy.init(args=args)
    voice_processing_example = VoiceProcessingExample()

    try:
        rclpy.spin(voice_processing_example)
    except KeyboardInterrupt:
        voice_processing_example.get_logger().info('Shutting down Voice Processing Example')
    finally:
        # Clean up audio resources
        if voice_processing_example.recording:
            voice_processing_example.stop_recording()
        voice_processing_example.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()