# Bluetooth DSP Speaker with 8-Band Parametric EQ

A real-time digital audio processing system that receives Bluetooth audio (A2DP) and applies an 8-band parametric equalizer with live FFT spectrum visualization.

## System Architecture
<img width="1264" height="962" alt="image" src="https://github.com/user-attachments/assets/c86cd0b2-8426-4963-933e-e8f650cec420" />

## Hardware

| Component | Model | Role |
|-----------|-------|------|
| MCU (DSP) | STM32F407VG Discovery | Audio filtering, FFT, EQ processing |
| Bluetooth | ESP32 DEVKIT (WROOM) | A2DP sink + UART bridge |
| DAC | Pmod I2S2 (ADAU1761) | Digital-to-analog conversion |
| Amplifier | TPA3116D2 | Class-D power amplifier |
| Speakers | Pioneer TS-F1035R (×2) | 4" full-range, 150W max |
| Power | 24V / 150W PSU | Powers amplifier |

## Pin Connections

### I2S (Audio Data)

| ESP32 | Signal | STM32 | Signal | Pmod I2S2 |
|-------|--------|-------|--------|-----------|
| GPIO26 | BCK | PB10 | I2S2_CK | Pin 9 (SCLK) |
| GPIO25 | WS | PB12 | I2S2_WS | Pin 8 (WS) |
| GPIO22 | DOUT | PC2 | I2S2_ext_SD | — |
| — | — | PC3 | I2S2_SD | Pin 10 (SDIN) |
| — | — | PC6 | I2S2_MCK | Pin 7 (MCLK) |

### UART (Control)

| PC | ESP32 | STM32 |
|----|-------|-------|
| COM5 (USB) | Serial (USB) | — |
| — | GPIO2 (TX) | PA3 (RX) |
| — | GPIO4 (RX) | PA2 (TX) |
| — | GND | GND |

### Clock Configuration

```
WS   = 44.1 kHz
SCLK = 44.1k × 2 × 32 = 2.82 MHz
MCLK = 44.1k × 256    = 11.3 MHz
```

## Features

- **8-Band Parametric EQ**: Peaking filters at 63, 125, 250, 500, 1k, 2k, 4k, 8k Hz with ±12 dB gain
- **Real-Time FFT Spectrum**: 1024-point FFT with 64-bin log-spaced display, showing pre and post-filter audio
- **Python Control GUI**: Dark-themed interface with vertical sliders, EQ response curve, and live spectrum overlay
- **EQ Presets**: FLAT, BASS+, TREBLE+, V-SHAPE, VOCAL, ROCK
- **Bluetooth A2DP**: Stream audio from any phone via Bluetooth (SBC codec)

## Project Structure

```
bluetooth-dsp-speaker/
├── stm32/
│   └── Bluetooth_stm32/          # STM32CubeIDE project
│       ├── Core/Src/main.c       # Audio processing, EQ filters, FFT collection
│       ├── Core/Src/freertos.c   # UART task, FFT task, EQ coefficient calculation
│       └── Core/Inc/             # Header files
├── esp32/
│   └── esp32_bluetooth.ino       # Bluetooth A2DP sink + UART bridge
├── python/
│   └── eight_band_eq_controller.py  # PyQt5 GUI for EQ control + spectrum display
├── docs/
│   └── bluetooth_speaker_dsp_diagram.pptx  # System block diagram
├── .gitignore
└── README.md
```

## Software Dependencies

### STM32
- STM32CubeIDE 1.19+
- FreeRTOS (CMSIS-RTOS v2)
- CMSIS-DSP library (arm_math.h)
- Preprocessor define: `ARM_MATH_CM4`
- Linker: `-larm_cortexM4lf_math`
- `configTOTAL_HEAP_SIZE = 32768`

### ESP32
- Arduino IDE
- [ESP32-A2DP library](https://github.com/pschatzmann/ESP32-A2DP) by Phil Schatzmann
- Board: ESP32 Dev Module

### Python
```bash
pip install PyQt5 pyqtgraph pyserial numpy
```

## How to Use

### 1. Flash STM32
- Open `stm32/Bluetooth_stm32/` in STM32CubeIDE
- Verify `configTOTAL_HEAP_SIZE = 32768` in `FreeRTOSConfig.h`
- Verify `ARM_MATH_CM4` is defined in preprocessor settings
- Clean → Build → Flash

### 2. Flash ESP32
- Open `esp32/esp32_bluetooth.ino` in Arduino IDE
- Select board: ESP32 Dev Module
- Upload

### 3. Connect Hardware
- Wire I2S and UART connections as shown in pin tables above
- Connect Pmod I2S2 to TPA3116D2 analog output
- Power amplifier with 24V supply

### 4. Run Python GUI
```bash
cd python
python eight_band_eq_controller.py
```
- Select COM port and click Connect
- Pair phone to "Kai_Speaker" via Bluetooth
- Play music and adjust EQ sliders

## UART Protocol

| Command | Example | Response | Description |
|---------|---------|----------|-------------|
| PING | `PING\n` | `OK:PONG\r\n` | Connection test |
| EQ | `EQ:3,6\n` | `OK:EQ\r\n` | Set band 3 (500Hz) to +6dB |
| FFT | — | `FFT:v0,v1,...;v0,v1,...\r\n` | 64 pre-filter bins ; 64 post-filter bins |

Band index: 0=63Hz, 1=125Hz, 2=250Hz, 3=500Hz, 4=1kHz, 5=2kHz, 6=4kHz, 7=8kHz

Gain range: -12 to +12 dB (integer)

## Screenshots

*Add screenshots of the Python GUI here*

## Technical Details

### Audio Processing Chain
```
Bluetooth (SBC) → ESP32 (16→32bit) → I2S → STM32 DMA → 8× Biquad EQ → I2S → ADAU1761 DAC → TPA3116D2 → Speakers
```

### EQ Filter Design
- Type: Peaking EQ (Robert Bristow-Johnson Audio EQ Cookbook)
- Implementation: Direct Form I biquad, 8 stages in series
- Q factor: 0.7 (wide bandwidth)
- Coefficients computed on STM32 using `powf`, `cosf`, `sinf`

### FFT Implementation
- 1024-point real FFT using CMSIS-DSP `arm_rfft_fast_f32`
- 512 frequency bins compressed to 64 display bins with log-spaced mapping
- Update rate: ~2 Hz (500ms interval)
- Magnitude scaled to 0-999 integer range

### FreeRTOS Tasks
| Task | Priority | Stack | Role |
|------|----------|-------|------|
| DefaultTask | Normal | 512B | Start DMA |
| UartTask | Low | 2048B | Parse commands, compute EQ coefficients |
| FFTTask | Low | 2048B | FFT computation, UART output |
| ButtonTask | Low | 2048B | Reserved for hardware button control |

## License

This project is for educational purposes.
