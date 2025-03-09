# Project Overview

This project involves taking a broken robot vacuum cleaner and rewiring all its sensors and motors into an external microcontroller. The goal is to repurpose the hardware and control it using a Raspberry Pi Pico RP2040 running FreeRTOS.

# Hardware

### **Microcontroller**: [RP2040-Zero](https://www.waveshare.com/wiki/RP2040-Zero)
### **Sensors and Motors**: Salvaged from a very old [Ropo Glass](https://web.archive.org/web/20150526031732/http://www.ropo.com.br/) robot vacuum
- 2 brushed DC motors with encoders
- Vacuum and brush motors *(I do not plan to use these)*
- Weight-on-Wheels switch
- 4 IR cliff sensors
- 5 IR distance sensors
- Bump sensors

# Software

As mentioned above, the RP2040 is running FreeRTOS with heap4 memory management.

I'm still working out the folder structure and CMake.

### Files
- **src/Ropo2040.c** -> Entry point of the code
- **inc/FreeRTOSConfig.h** -> FreeRTOS configuration file
- **lib/** -> External libraries folder, including the FreeRTOS kernel

The FreeRTOS kernel is included as a submodule, so make sure to clone the repository with the `--recurse-submodules` flag, or if you've already cloned it, use `git submodule init` and `git submodule update`.

# Current status

This is the first steps, currently the code is just some basic FreeRTOS functionality test on the RP2040.

# Future Plans*

- Integrate all sensors and motors with the RP2040-Zero
- Develop control algorithms for autonomous operation
- Implement additional features and improvements
- Create a custom PCB

_*This is just a hobby project that I'm working on for fun, so I expect to abandon it unfinished at any time lol_
