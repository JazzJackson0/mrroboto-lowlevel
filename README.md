

sudo openocd -f interface/cmsis-dap.cfg -c "adapter speed 5000" -f target/rp2040.cfg -s tcl


Hardware
--------

Raspberry Pi Pico
Pins: 
    IN
    -------------------------
    PWM Val IN [From Pi]
    + uart rx: GP9 (12)

    Encoder IN [From Sensor]
    + left encoder pins: GP18 (24), GP19 (25)
    + right encoder pins: GP13 (17), GP12 (16)
    
    IMU IN [From Sensor]
    + i2c1 sda: GP4 (6)
    + i2c1 scl: GP5 (7)
    
    OUT
    --------------------------

    Encoder & IMU OUT [To Pi]
    + i2c1 sda: GP26 (31)
    + i2c1 scl: GP27 (32)

    PWM OUT [To Motor]
    + left motor pin: GP20 (26)
    + right motor pin: GP11 (15)
    
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







