


Hardware
--------

Raspberry Pi Pico
Pins: 
    IN
    -------------------------
    PWM Val IN [From Pi]
    + uart rx: GP9 (12)

    Encoder IN [From Sensor]
    + left encoder pin: GP18 (24)
    + right encoder pin: GP13 (17)
    
    IMU IN [From Sensor]
    + i2c1 sda: GP4 (6)
    + i2c1 scl: GP5 (7)
    
    OUT
    --------------------------

    Encoder & IMU OUT [To Pi]
    + i2c1 sda: GP26 (31)
    + i2c1 scl: GP27 (32)

    PWM OUT [To Motor]
    + left motor pin: GP19 (25)
    + right motor pin: GP12 (16)
    
    Motor Direction Pins
    Left Motor Dir Pins: GP16, GP17
    Right Motor Dir Pins: GP14, GP15
    --------------------------
    
    Power
    + vsys (From Pi 4): (39)
    + gnd: (38)


Add CMSIS-DSP to build
----------------------
cmake .. -DCMSISCORE=cmsis-core-master -DCMSISDSP=CMSIS-DSP-main







