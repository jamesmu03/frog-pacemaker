# Real-Time Cardiac Pacemaker (ESP32)

A performance-critical embedded system designed for real-time cardiac stimulus delivery and physiological monitoring on the ESP32 platform. This project implements advanced signal processing and deterministic control logic using a dual-core RTOS architecture.

## Technical Architecture

### Real-Time Multitasking
The system utilizes **FreeRTOS** to ensure deterministic execution of time-sensitive tasks across the ESP32's dual-core architecture:
- **Core 0 (Control Logic)**: Executes the high-priority Pacemaker Task, governing the system state machine and precise pacing pulse delivery based on Lower Rate Interval (LRI) constraints.
- **Core 1 (Signal Processing)**: Manages continuous high-frequency ADC sampling (400Hz) and executes the digital signal processing pipeline.

### Digital Signal Processing
- **Pan-Tompkins Algorithm**: Real-time implementation of the Pan-Tompkins pipeline for robust QRS complex detection, including bandpass filtering, derivative calculations, and squaring functions.
- **Dynamic Thresholding**: Implemented a closed-loop feedback system utilizing a DAC to dynamically set comparator thresholds. Thresholds are recalculated based on a sliding-window average and standard deviation of historical R-peak amplitudes.

### Deterministic State Machine
The firmware follows a rigorous state-transition model to ensure system stability:
- **Initialization**: Sensor calibration and parameter quantification.
- **Acquisition**: Non-blocking ADC sampling and interrupt-syncing via binary semaphores.
- **Processing**: QRS detection and heart rate quantification.
- **Pacing**: Precision stimulus delivery (Chronaxie-timed) via GPIO-driven pulses.

## Hardware Interfacing

- **Microcontroller**: ESP32 (WROOM-32)
- **Peripherals**:
    - **ADC**: High-resolution ECG acquisition.
    - **DAC**: Dynamic thresholding control via analog feedback.
    - **I2C**: OLED (SSD1306) telemetry interface for real-time diagnostics.
- **PCB Design**: Custom KiCad schematics defining the signal conditioning and power management stages.

## Visual Assets

- [System Flow Chart](flow_chart_new_new.puml)
- [Circuit Schematic](Schematic.svg)
- [ESP32 Pinout Reference](ESP32-Pinout-1.jpg)

## Component Overview

- `src/main.cpp`: RTOS task definitions and state machine implementation.
- `lib/`: Optimized C++ classes for the Pan-Tompkins detector and ancillary DSP functions.
- `platformio.ini`: Bare-metal build configuration for the ESP32-WROOM environment.
