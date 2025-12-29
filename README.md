# Mr. Roboto (Low Level)

Firmware for a Raspberry Pi Pico–based differential drive robot running FreeRTOS.  
Includes motor control, encoder feedback, IMU sensing, and UART communication with a Raspberry Pi 4.


---

## Project Overview
TODO: Not 100% Correct

This project implements low-level control for a differential drive robot:
- Left and right DC motors with PWM speed control
- Quadrature encoder feedback for wheel odometry
- IMU for inertial sensing
- UART interface to a Raspberry Pi 4 for high-level control
- FreeRTOS for real-time task management

---

## Hardware Setup

### Microcontroller
- **Board:** Raspberry Pi Pico (RP2040)
- **Clock:** 125 MHz (default)

### Power
- **VSYS (from Raspberry Pi 4):** Pin 39  
- **GND:** Pin 38

### UART (Pi ↔ Pico)
| Function | Pico Pin | Notes |
|-----------|-----------|-------|
| UART RX (from Pi) | GP9 (Pin 12) | Receives Motor Data Packets |

### I2C (IMU ↔ Pico)
| Signal | Pico Pin | Notes |
|---------|-----------|-------|
| SDA | GP4 (Pin 6) | I2C1 SDA |
| SCL | GP5 (Pin 7) | I2C1 SCL |

### (Encoders ↔ Pico)
| Wheel | Channel A | Channel B |
|--------|------------|-----------|
| Left | GP18 (Pin 24) | GP19 (Pin 25) |
| Right | GP13 (Pin 17) | GP12 (Pin 16) |

### I2C Encoder & IMU (Pico ↔ Pi)
| Signal | Pi Pin | Notes |
|---------|-----------|-------|
| SDA | GPIO 26 (Pin 31) | I2C1 SDA |
| SCL | GPIO 27 (Pin 32) | I2C1 SCL |

### (Pico ↔ Motors): PWM + Direction
| Wheel | PWM | Dir 1 | Dir 2 |
|--------|------|-------|-------|
| Left | GP20 (Pin 26) | GP16 (Pin 21) | GP17 (Pin 22) |
| Right | GP11 (Pin 15) | GP14 (Pin 19) | GP15 (Pin 20) |

---

## Software Components
TODO: Not 100% Correct
- **FreeRTOS Kernel** – Task scheduling, timing, synchronization  
- **Motor Driver Module** – PWM generation and direction control  
- **Encoder Module** – Quadrature decoding for odometry  
- **IMU Module** – I2C communication for orientation data  
- **UART Interface** – Communication with Raspberry Pi  
- **Main Control Task** – Updates motion commands and sensor fusion  

---

## Building with CMake

**Add CMSIS-DSP to build**: `cmake .. -DCMSISCORE=cmsis-core-master -DCMSISDSP=CMSIS-DSP-main`


**Debug (Terminal 1)**: `sudo openocd -f interface/cmsis-dap.cfg -c "adapter speed 5000" -f target/rp2040.cfg -s tcl`


**View OUTPUT STREAM**: view stream coming from the tested pico through the pico probe's UART connection 

`sudo minicom -D /dev/ttyACM0 -b 115200`


