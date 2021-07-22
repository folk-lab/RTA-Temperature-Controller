# RTA-Temperature-Controller

Technical specifics, code, schematics and operating instructions for the RTA temperature controller.  Currently setup to be used with a Osram FNT 64656 HLX 275W 24V tungsten halogen lamp.  

### Components

- Staco Energy Type 291 Variable transformer
- Amveco Magnetics AA56252-024 Toroidal transformer
- FOTEK SSR-25 DA (3-32VDC Input, 25A @ 24-380 VAC Load)
- Arduino Pro Micro (ATMEGA32U4, 5V/16MHZ)
- Adafruit OLED Display (I2C, 0.96" (24.38mm) diagonal screen size, 128 x 64 pixels)
- MAX6675 K-Type thermocouple module

### Operating Instructions

#### Up Toggle Position - Manual

- This allows the user to operate the temperature controller as before.  Simply use the variable transfomer to adjust the output.


#### Down Toggle Position - PID controlled

- This position allows the user to control the output using pulse width modulated PID.
