# GitHub Copilot Instructions for LCAV Audio DSP Labs

This is an embedded audio signal processing project for Raspberry Pi Pico (RP2350), utilizing the Pico SDK and CMSIS-DSP library.

## Project Structure & Architecture

*   **`firmware_core/`**: Contains the system chassis. Handles USB (TinyUSB), I2S, DMA, and the main run loop.
    *   **`main.c`**: Initializes hardware and runs the superloop calling `audio_process()`.
    *   **`audio_proc.h`**: Defines the interface that each lab must implement (`audio_proc_init`, `audio_process`).
    *   **`ring_buffer.h`**: Custom ring buffer implementation used for audio streams.
*   **`labX_[name]/`**: Contains the specific implementation for a lab assignment.
    *   **`audio_proc.c`**: The *primary* file for implementing DSP logic.
*   **`CMakeLists.txt`**: Root build configuration. Controls which lab is currently active via `LAB_ID`.

## Critical Workflows

### 1. building & Switching Labs
To switch between labs, you must edit `CMakeLists.txt`:
```cmake
set(LAB_ID 5 CACHE STRING "Selects the lab to build")
```
*   **Build**: Run the VS Code task **"Compile Project"** (uses Ninja).
*   **Flash**: Run the VS Code task **"Run Project"** (uses `picotool`) or **"Flash"** (uses OpenOCD).

### 2. Implementation Pattern (`audio_proc.c`)
Each lab must implement:
*   `void audio_proc_init()`: Called once at startup. Initialize filters, states, and buffers here.
*   `void audio_process()`: Called repeatedly in the main loop when data is available.

**Data Flow**:
1.  Read from `g_i2s_to_proc_buffer` (Input from I2S/Mic).
2.  Process audio (Block processing recommended).
3.  Write to `g_proc_to_usb_buffer` (Output to USB/Speaker).

**Example Loop**:
```c
// Typical pattern in audio_process()
while (rb_read_available(&g_i2s_to_proc_buffer) >= BLOCK_SIZE && 
       rb_write_available(&g_proc_to_usb_buffer) >= BLOCK_SIZE) {
    // 1. Pop data
    rb_pop(&g_i2s_to_proc_buffer, (uint8_t*)input_buffer, BLOCK_SIZE * sizeof(int16_t));
    
    // 2. Convert to float/process
    // ... CMSIS-DSP calls ...
    
    // 3. Push data
    rb_push(&g_proc_to_usb_buffer, (uint8_t*)output_buffer, BLOCK_SIZE * sizeof(int16_t));
}
```

## Coding Conventions & Dependencies

*   **Math**: Use **CMSIS-DSP** (`arm_math.h`) for all signal processing (FFT, FIR, IIR, Matrix).
    *   Note: `arm_math_types.h` is often needed.
    *   Target is usually `float32_t`.
*   **Hardware**: Use standard **Pico SDK** headers (`pico/stdlib.h`, `hardware/dma.h`, `pico/multicore.h`).
*   **Ring Buffers**: Use the provided `rb_*` macros and functions from `firmware_core/ring_buffer.h` (e.g., `rb_init_static_size_aligned`).

## Debugging

*   **GPIO Profiling**:
    *   **GPIO 14 (`DBG_LOOP_PIN`)**: Toggles high during `audio_process()` execution. Use logic analyzer to measure CPU load.
    *   **GPIO 15 (`DBG_AUDIO_PIN`)**: Available for custom timing debugging.
*   **LED**: The onboard LED toggles every 500ms to indicate liveness.
*   **Serial**: `stdio_uart_init()` is called; use UART for text logging if needed, but avoid in the audio path to prevent dropouts.
