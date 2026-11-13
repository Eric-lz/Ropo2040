# Project Overview

This project involves taking a broken robot vacuum cleaner and rewiring all its sensors and motors into an external microcontroller. The goal is to repurpose the hardware and control it using a Raspberry Pi Pico RP2040 running FreeRTOS.

# Hardware

### **Microcontroller**: [RP2040-Zero](https://www.waveshare.com/wiki/RP2040-Zero)
### **Motor Driver**: BTS7960 (HW-039) H-Bridge
### **Current Sensor**: INA219 (I2C)
### **Sensors and Motors**: Salvaged from a very old [Ropo Glass](https://web.archive.org/web/20150526031732/http://www.ropo.com.br/) robot vacuum
- 2 brushed DC geared motors with optical encoders
- Vacuum and brush motors *(I do not plan to use these)*
- Weight-on-Wheels switch
- 4 IR cliff sensors
- 5 IR distance sensors
- Bump sensors

# Software

The firmware is written in C/C++ using the Pico SDK and runs FreeRTOS (heap4).

### Structure
- **src/Ropo2040.cpp** -> Application entry point and FreeRTOS task setup
- **src/control.cpp** -> Closed-loop wheel position controller and motion profiling
- **src/encoder.cpp** & **src/edge_counter.pio** -> PIO-based wheel encoder counter with glitch filtering
- **src/motor.cpp** -> PWM motor driver control
- **src/current.cpp** -> INA219 current sensor reading
- **src/tests.cpp** -> Automated movement and accuracy tests
- **inc/config.h** -> Pinout and hardware configuration constants
- **lib/** -> External libraries, including the FreeRTOS kernel and INA219 driver

The FreeRTOS kernel is included as a submodule, so make sure to clone the repository with the `--recurse-submodules` flag, or if you've already cloned it, use `git submodule init` and `git submodule update`.

# Current Status

Single-wheel closed-loop position control is working reliably:
- PIO state machine counting both encoder edges with hardware glitch filtering
- 100 Hz motion profile and PID + feedforward position control loop
- Multi-turn accuracy tests and live serial telemetry

# Future Plans*

- Expand control to both drive wheels (differential drive)
- Integrate cliff, bump, and distance sensors with the RP2040-Zero
- Develop control algorithms for autonomous navigation
- Create a custom PCB

_*This is just a hobby project that I'm working on for fun, so I expect to abandon it unfinished at any time._
