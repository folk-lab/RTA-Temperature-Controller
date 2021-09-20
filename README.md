# RTA-Temperature-Controller

Technical specifics, code, schematics and operating instructions for the RTA temperature controller.  Currently, setup to be used with a Osram FNT 64656 HLX 275W 24V tungsten halogen lamp.  

## Components

- Staco Energy Type 291 Variable transformer
- Amveco Magnetics AA56252-024 Toroidal transformer
- FOTEK SSR-25 DA (3-32VDC Input, 25A @ 24-380 VAC Load) ... Omega SSRL240DC25 will also work.
- Arduino Pro Micro (ATMEGA32U4, 5V/16MHZ)
- Adafruit OLED Display (I2C, 0.96" (24.38mm) diagonal screen size, 128 x 64 pixels)
- MAX31855K K-Type thermocouple module (v1 used the MAX6675)

## Operating Instructions

There are two modes of operation. 

### Up Toggle Position - Manual
Allows the user to operate the temperature controller as before.  Simply use the variable transformer to adjust the output.

### Down Toggle Position - PID Control
Automates an annealing sequence by using the Arduino control the relay switching.

## Programming 

The steps in an annealing schedule is specified by creating `HeatingStep` objects, and pushing them to a `StackArray`. A `HeatingStep` object is specified by supplying the following parameters
1. Setpoint temperature in degrees Celsius 
2. The P coefficient - note that we used **[proportional on measurement](http://brettbeauregard.com/blog/2017/06/introducing-proportional-on-measurement/)**
3. The I coefficient 
4. The D coefficient 
5. The number of seconds to hold at the setpoint after ramping 
6. `delta_t` (in degrees Celsius) - in a ramp up step, as long the setpoint is at least `delta_t` higher than the measured temperature, the relay remains fully closed 


> **Tip:** specifying small `delta_t` increases the speed of the ramp up 

For example: 
```cpp
// heat to 445, using kp = 3.8, ki = 0.9, kd = 0, hold for 120 seconds, with a delta_t of 5 degrees celcius
HeatingStep step2(445, 3.8, 0.9, 0.0, 120, 5.0);
```

For every `HeatingStep` object in the `StackArray`, we first 
1. Ramp to the setpoint temperature
2. Start a timer, and hold at the setpoint for the specified number of seconds 

When we are done with a `HeatingStep`, we `pop` the `HeatingStep` from the `StackArray` object. The algorithm repeats until the `StackArray` is empty. At this point, the display will show 
```cpp
"COMPLETE"
```

### Example 
Let's consider the following annealing schedule, starting at room temperature:
1. 330 degrees Celsius, 2 minutes 
2. 445 degrees Celsius, 2 minutes
3. Cool down to 50 degrees Celsius, at which point we can remove the sample 

This would be specified by the following section in the RTA code:
```cpp 
//...

/*
Modify below 
*/
// Temperature (in degrees), kP, kI, kD, seconds to hold temperature at, delta_t 
HeatingStep step0(360, 4.00, 1.2, 0.0, 120, 6.0); // Set knob to 60% full power
HeatingStep step1(445, 3.8, 0.9, 0.0, 120, 5.0); 
HeatingStep step2(50, 0.0, 0.0, 0.0, 1, 0);
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
}
//...
```

### Pseudo Code
Every annealing step essentially consists of the following 4 `while` loops.

```python
annealing_step: 
	while "(target temperature - measured temperature) is >= delta_t"
		"Fully OPEN the relay" 

	while "(target temperature - measured temperature) is <= delta_t"
		"Fully CLOSE the relay"

	while "measured temperature is within 1 C of the target temperature"
		"Start the PID algorithm to control the relay switching"

	# at this point, should already be super close to the target temperature

	while "held time is less than the required holding time"
		"Use the PID algorithm to hold at the target temperature"
complete
```