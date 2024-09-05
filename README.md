


Hardware
--------

Raspberry Pi Pico
Pins:
    -------------------------
    PWM Val IN [From Pi]
    + uart tx: GP0 (1)

    PWM OUT [To Motor]
    + left motor pin: GP17 (22)
    + right motor pin: GP14 (19)
    --------------------------
    Encoder IN [From Sensor]
    + left encoder pin: GP16 (21)
    + right encoder pin: GP15 (20)
    
    IMU IN [From Sensor]
    + i2c1 sda: GP4 (6)
    + i2c1 scl: GP5 (7)
    + [Other Pins receiving from Sensor GP8 (11), GP9 (12), GP10 (14)]

    Encoder & IMU OUT [To Pi]
    + i2c1 sda: GP26 (31)
    + i2c1 scl: GP27 (32)
    --------------------------
    Power
    + vsys (From Pi 4): (39)
    + gnd: (38)
