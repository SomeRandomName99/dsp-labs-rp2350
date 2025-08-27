# I2S Microphone to CDC Passthrough for Pico 2 W

This project captures audio from an I2S microphone and sends it over USB CDC to your computer. It's designed for the Raspberry Pi Pico 2 W.

## Features

- **Audio Capture**: Captures audio at 16kHz, 16-bit, mono
- **USB CDC Output**: Sends audio data over USB to your computer
- **Real-time Processing**: Continuous audio streaming with minimal latency
- **Configurable**: Easy to modify sample rate, bit depth, and buffer size

## Hardware Requirements

- Raspberry Pi Pico 2 W
- I2S microphone (e.g., INMP441, SPH0645LM4H-B, etc.)
- Breadboard and jumper wires

## Pin Connections

Connect your I2S microphone to the Pico 2 W as follows:

| I2S Microphone | Pico 2 W Pin | Description |
|----------------|---------------|-------------|
| VCC            | 3V3           | Power supply |
| GND            | GND           | Ground |
| SD             | GPIO 20       | Data line |
| SCK            | GPIO 21       | Bit clock |
| WS             | GPIO 22       | Word select/LR clock |

## Building the Project

1. **Prerequisites**: Make sure you have the Pico SDK installed and configured
2. **Build**: Run the following commands:
   ```bash
   mkdir build
   cd build
   cmake ..
   make
   ```
3. **Flash**: Connect your Pico 2 W and flash the binary:
   ```bash
   picotool load audioPassThrough.uf2
   ```

## Usage

1. **Connect Hardware**: Wire up your I2S microphone according to the pin connections above
2. **Flash and Run**: Build and flash the project to your Pico 2 W
3. **Connect USB**: The Pico will appear as a USB CDC device
4. **Monitor Output**: Use a serial monitor or terminal to see the audio data

## Audio Data Format

The audio data is sent over USB CDC in the following format:

1. **Header**: 4-byte buffer size (little-endian)
2. **Audio Data**: Raw 16-bit PCM samples (little-endian)
3. **Sample Rate**: 16kHz
4. **Bit Depth**: 16-bit
5. **Channels**: Mono (1 channel)

## Customization

### Sample Rate
Modify `SAMPLE_RATE` in `audioPassThrough.c`:
```c
#define SAMPLE_RATE 44100  // Change to 44.1kHz
```

### Buffer Size
Adjust `BUFFER_SIZE` for different latency/throughput trade-offs:
```c
#define BUFFER_SIZE 2048   // Larger buffer, higher latency
```

### Pin Configuration
Change the I2S pins if needed:
```c
#define I2S_DATA_PIN 18    // Different data pin
#define I2S_BCLK_PIN 19    // Different bit clock pin
#define I2S_LRCLK_PIN 20   // Different word select pin
```

## Current Implementation Notes

**Important**: The current implementation is a simplified version that demonstrates the concept. For actual I2S microphone capture, you'll need to:

1. **Implement Proper I2S Timing**: The current version uses basic GPIO reading
2. **Add I2S Clock Generation**: Generate proper BCLK and LRCLK signals
3. **Handle I2S Protocol**: Properly decode the I2S data stream
4. **Use PIO or Hardware I2S**: For better performance and accuracy

## Next Steps for Full I2S Implementation

To implement proper I2S microphone capture:

1. **Use PIO State Machine**: Create a PIO program for I2S capture
2. **Generate I2S Clocks**: Use PIO to generate BCLK and LRCLK
3. **Proper Data Sampling**: Sample data on the correct clock edges
4. **DMA Integration**: Use DMA for efficient data transfer

## Troubleshooting

### No Audio Data
- Check pin connections
- Verify microphone power supply
- Check USB CDC connection

### Poor Audio Quality
- Ensure stable power supply
- Check for electrical interference
- Verify I2S timing

### Build Errors
- Ensure Pico SDK is properly installed
- Check CMake configuration
- Verify toolchain setup

## License

This project is open source. Feel free to modify and distribute.

## Contributing

Contributions are welcome! Please feel free to submit issues and pull requests.
