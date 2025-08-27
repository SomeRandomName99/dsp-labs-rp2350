#!/usr/bin/env python3
"""
Audio Receiver for Pico 2 W I2S Microphone to CDC Passthrough

This script receives audio data from the Pico over USB CDC and can:
- Save it to a WAV file
- Play it in real-time
- Analyze the audio data
"""

import serial
import struct
import wave
import numpy as np
import time
import argparse
from pathlib import Path

class AudioReceiver:
    def __init__(self, port=None, baudrate=115200, timeout=1):
        """Initialize the audio receiver"""
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.serial_conn = None
        self.sample_rate = 16000
        self.sample_width = 2  # 16-bit = 2 bytes
        self.channels = 1
        
    def find_pico_port(self):
        """Automatically find the Pico CDC port"""
        import glob
        
        # Common patterns for Pico CDC ports
        patterns = [
            '/dev/ttyACM*',      # Linux
            '/dev/tty.usbmodem*', # macOS
            'COM*'               # Windows
        ]
        
        for pattern in patterns:
            ports = glob.glob(pattern)
            for port in ports:
                try:
                    # Try to open the port and check if it's our Pico
                    with serial.Serial(port, self.baudrate, timeout=1) as test_conn:
                        time.sleep(2)  # Wait for device to initialize
                        if test_conn.in_waiting > 0:
                            data = test_conn.read(test_conn.in_waiting)
                            if b'I2S Microphone to CDC Passthrough' in data:
                                print(f"Found Pico on port: {port}")
                                return port
                except:
                    continue
        
        return None
    
    def connect(self):
        """Connect to the Pico"""
        if self.port is None:
            self.port = self.find_pico_port()
            
        if self.port is None:
            raise Exception("Could not find Pico CDC port. Please specify manually.")
            
        print(f"Connecting to {self.port}...")
        self.serial_conn = serial.Serial(
            port=self.port,
            baudrate=self.baudrate,
            timeout=self.timeout
        )
        
        # Wait for device to be ready
        time.sleep(2)
        
        # Clear any initial data
        self.serial_conn.reset_input_buffer()
        
        print("Connected to Pico!")
        return True
    
    def receive_audio_buffer(self):
        """Receive one audio buffer from the Pico"""
        if self.serial_conn is None:
            raise Exception("Not connected to Pico")
            
        # Read buffer size header (4 bytes)
        header = self.serial_conn.read(4)
        if len(header) != 4:
            return None
            
        buffer_size = struct.unpack('<I', header)[0]
        expected_samples = buffer_size // self.sample_width
        
        # Read audio data
        audio_data = self.serial_conn.read(buffer_size)
        if len(audio_data) != buffer_size:
            return None
            
        # Convert to 16-bit samples
        samples = struct.unpack(f'<{expected_samples}h', audio_data)
        return np.array(samples, dtype=np.int16)
    
    def save_to_wav(self, filename, duration_seconds=10):
        """Save received audio to a WAV file"""
        print(f"Recording {duration_seconds} seconds to {filename}...")
        
        # Calculate number of buffers needed
        samples_per_buffer = 1024  # From the C code
        buffers_needed = int((self.sample_rate * duration_seconds) / samples_per_buffer)
        
        # Prepare WAV file
        with wave.open(filename, 'wb') as wav_file:
            wav_file.setnchannels(self.channels)
            wav_file.setsampwidth(self.sample_width)
            wav_file.setframerate(self.sample_rate)
            
            buffer_count = 0
            start_time = time.time()
            
            while buffer_count < buffers_needed and (time.time() - start_time) < duration_seconds:
                audio_buffer = self.receive_audio_buffer()
                if audio_buffer is not None:
                    # Write to WAV file
                    wav_file.writeframes(audio_buffer.tobytes())
                    buffer_count += 1
                    
                    # Progress indicator
                    if buffer_count % 10 == 0:
                        elapsed = time.time() - start_time
                        print(f"Recorded {buffer_count}/{buffers_needed} buffers ({elapsed:.1f}s)")
                else:
                    print("Warning: Received incomplete audio buffer")
                    time.sleep(0.1)
        
        print(f"Recording complete! Saved to {filename}")
    
    def real_time_monitor(self, duration_seconds=30):
        """Monitor audio in real-time"""
        print(f"Monitoring audio for {duration_seconds} seconds...")
        print("Press Ctrl+C to stop early")
        
        start_time = time.time()
        buffer_count = 0
        
        try:
            while (time.time() - start_time) < duration_seconds:
                audio_buffer = self.receive_audio_buffer()
                if audio_buffer is not None:
                    # Calculate some basic statistics
                    rms = np.sqrt(np.mean(audio_buffer.astype(np.float32)**2))
                    peak = np.max(np.abs(audio_buffer))
                    
                    buffer_count += 1
                    elapsed = time.time() - start_time
                    
                    print(f"Buffer {buffer_count}: RMS={rms:.1f}, Peak={peak}, Time={elapsed:.1f}s")
                else:
                    print("Warning: Received incomplete audio buffer")
                    time.sleep(0.1)
                    
        except KeyboardInterrupt:
            print("\nMonitoring stopped by user")
        
        print(f"Monitoring complete! Processed {buffer_count} buffers")
    
    def close(self):
        """Close the connection"""
        if self.serial_conn:
            self.serial_conn.close()
            print("Connection closed")

def main():
    parser = argparse.ArgumentParser(description='Receive audio from Pico 2 W I2S microphone')
    parser.add_argument('--port', '-p', help='Serial port (auto-detect if not specified)')
    parser.add_argument('--baudrate', '-b', type=int, default=115200, help='Baud rate')
    parser.add_argument('--mode', '-m', choices=['record', 'monitor'], default='monitor',
                       help='Mode: record (save to WAV) or monitor (real-time)')
    parser.add_argument('--output', '-o', default='audio_capture.wav', help='Output WAV file (for record mode)')
    parser.add_argument('--duration', '-d', type=int, default=10, help='Duration in seconds')
    
    args = parser.parse_args()
    
    # Create output directory if needed
    if args.mode == 'record':
        output_path = Path(args.output)
        output_path.parent.mkdir(parents=True, exist_ok=True)
    
    # Initialize receiver
    receiver = AudioReceiver(port=args.port, baudrate=args.baudrate)
    
    try:
        # Connect to Pico
        receiver.connect()
        
        # Run in specified mode
        if args.mode == 'record':
            receiver.save_to_wav(args.output, args.duration)
        else:
            receiver.real_time_monitor(args.duration)
            
    except Exception as e:
        print(f"Error: {e}")
    finally:
        receiver.close()

if __name__ == '__main__':
    main()
