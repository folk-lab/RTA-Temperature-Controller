# RTA-Temperature-Controller

Technical specifics, code, schematics and operating instructions for the RTA temperature controller.  Currently setup to be used with a Osram FNT 64656 HLX 275W 24V tungsten halogen lamp.  

## Components

- Staco Energy Type 291 Variable transformer
- Amveco Magnetics AA56252-024 Toroidal transformer
- FOTEK SSR-25 DA (3-32VDC Input, 25A @ 24-380 VAC Load)
- Arduino Pro Micro (ATMEGA32U4, 5V/16MHZ)
- Adafruit OLED Display (I2C, 0.96" (24.38mm) diagonal screen size, 128 x 64 pixels)
- MAX6675 K-Type thermocouple module

## Operating Instructions

There are two modes of operation. 

### Up Toggle Position - Manual
Allows the user to operate the temperature controller as before.  Simply use the variable transfomer to adjust the output.

### Down Toggle Position - PID Control
Automates an annealing sequence. 

## Programming 

The steps in an annealing schedule is specified by creating `HeatingStep` objects, and pushing them to a `StackArray`. A `HeatingStep` object is specified by supplying the following parameters
1. Setpoint temperature in degress celius 
2. The $P$ coefficient in the PID algorithm 
3. The $I$ coefficient 
4. The $D$ coefficient 
5. The number of seconds to hold at the setpoint after ramping 

For example,
```cpp
# heat to 445, using kp = 3.8, ki = 0.9, kd = 0, and hold for 120 seconds
HeatingStep step2(445, 3.8, 0.9, 0.0, 120);
```

For every `HeatingStep` object in the `StackArray`, we first 
1. Ramp to the setpoint temperature
2. Start a timer, and hold at the setpoint for the specified number of seconds 

When we are done with a `HeatingStep`, we `pop` the `HeatingStep` from the `StackArray` object. The algorithm repeats until the `StackArray` is empty. At this point, the display will show 
```cpp
"QDEV FOREVER!!"
```

### Example 
Let's consider the following annealing schedule, starting at room temperature:
1. 330 degress celcius, 2 minutes 
2. 445 degrees celcius, 2 minutes
3. Cool down to 50 degress celcius, at which point we can remove the sample 

This would be specifed by the following section in the RTA code:
```cpp 
//...
//...

/*
Modify below 
*/
// Temperature [C], kP, kI, kD, Seconds to Hold Temperature At
HeatingStep step1(330, 3.8, 0.9, 0.0, 120); // Set knob to 60% full power
HeatingStep step2(445, 3.8, 0.9, 0.0, 120);
HeatingStep step3(50, 0, 5.0, 0.0, 1);
/*
Modify above
*/

StackArray<HeatingStep> heating_schedule;

void PID_fn(void);
void reset_display(void);
void set_pid_tune(double, double, double);

void setup()
{
    // We push the various heating steps onto the heating_schedule stack.
    // Remember that a stack data structure follows "last in, first out"
    /*
	Modify below 
	*/
	heating_schedule.push(step3);
	heating_schedule.push(step2);
	heating_schedule.push(step1);
	/*
	Modify above
	*/
    //...
    //...
}
//...
//...
```