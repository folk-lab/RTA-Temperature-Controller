#include <SPI.h>
#include <Wire.h>

// libraries that likely need to be downloaded in library manager
#include <PID_v1.h>
#include <max6675.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// additional libraries included with the file
#include <StackArray.h>
#include "src/Bitmap.h"
#include "src/HeatingSchedule.h"

#define SSR_PIN 9
#define OLED_reset_PIN 7

MAX6675 thermocouple(10, 16, 14); // (SCK pin, CS pin, SO pin)
Adafruit_SSD1306 display(128, 64, &Wire, 7);

// Ruiheng: I ripped this from our FastDAC code!
typedef struct PIDparam
{
	bool forward_dir = true;
	double Input = 0.0;
	double Output = 0.0;
	double Setpoint = 0.0;
	double kp = 0.1;
	double ki = 1.0;
	double kd = 0.0;
} PIDparam;

PIDparam g_pidparam[1];

PID myPID(&(g_pidparam[0].Input),
		  &(g_pidparam[0].Output),
		  &(g_pidparam[0].Setpoint),
		  g_pidparam[0].kp,
		  g_pidparam[0].ki,
		  g_pidparam[0].kd,
		  P_ON_M,
		  DIRECT);

/*
Modify below 
*/
// Temperature [C], kP, kI, kD, Seconds to Hold Temperature At
HeatingSchedule step1(330, 4.1, 0.9, 0.0, 120);
HeatingSchedule step2(445, 4.1, 0.9, 0.0, 120);
HeatingSchedule step3(50, 0, 5.0, 0.0, 1);
/*
Modify above
*/

StackArray<HeatingSchedule> schedule_stack;

void PID_fn(void);
void cool_fn(void);
void reset_display(void);
void set_pid_tune(double, double, double);

void setup()
{
	/*
	Modify below 
	*/
	schedule_stack.push(step3);
	schedule_stack.push(step2);
	schedule_stack.push(step1);
	/*
	Modify above
	*/

	pinMode(SSR_PIN, OUTPUT);
	digitalWrite(SSR_PIN, LOW);

	g_pidparam[0].Input = thermocouple.readCelsius();
	myPID.SetMode(AUTOMATIC);
	myPID.SetOutputLimits(0, 255); // although the function defaults to 0 to 255, we call this anyway to be safe

	display.begin(SSD1306_SWITCHCAPVCC, 0x3D);
	display.setTextColor(SSD1306_WHITE);
	display.setTextSize(2);
	display.clearDisplay();
	display.setCursor(0, 0);
	delay(100);
	display.drawBitmap(0, 0, myBitmap, 125, 65, WHITE);
	delay(100);
	display.display();
	delay(10000);
	reset_display();
	delay(100);
}

void loop()
{
	if (!schedule_stack.isEmpty())
	{
		PID_fn();
		// after ramping, holding
		schedule_stack.pop(); // pop the last entry from the stack
	}
	else
	{
		reset_display();
		display.println("QDEV FOREVER!!");
		display.display();
	}
}

void PID_fn(void)
{
	double setpoint = schedule_stack.peek().setpoint;
	double kp = schedule_stack.peek().proportional;
	double ki = schedule_stack.peek().integral;
	double kd = schedule_stack.peek().derivative;
	double ht = schedule_stack.peek().hold_time;
	double T = thermocouple.readCelsius();

	set_pid_tune(kp, ki, kd);
	g_pidparam[0].Setpoint = setpoint;

	// ramping sequence
	while (abs(setpoint - T) > 1.0)
	{
		delay(200);
		T = thermocouple.readCelsius();
		g_pidparam[0].Input = T;
		myPID.Compute();
		analogWrite(SSR_PIN, g_pidparam[0].Output);
		reset_display();

		display.println(String(T, 2) + " C");
		display.println("Ramp: " + String(setpoint, 2));
		display.display();
	}

	unsigned long start_time = millis(); // in ms

	// holding
	while ((millis() - start_time) / 1000.0 < ht)
	{
		delay(200);
		T = thermocouple.readCelsius();
		g_pidparam[0].Input = T;
		myPID.Compute();
		analogWrite(SSR_PIN, g_pidparam[0].Output);
		reset_display();
		display.println(String(T, 2) + " C");
		display.println("Hold: " + String((millis() - start_time) / 1000.0, 1) + " s");
		display.display();
	}
}

void set_pid_tune(double kp, double ki, double kd)
{
	g_pidparam[0].kp = kp;
	g_pidparam[0].ki = ki;
	g_pidparam[0].kd = kd;
	myPID.SetTunings(g_pidparam[0].kp, g_pidparam[0].ki, g_pidparam[0].kd);
}

void reset_display(void)
{
	display.clearDisplay();
	display.setCursor(0, 16);
}

void append_to_display(String message)
{
	display.println(message);
}
