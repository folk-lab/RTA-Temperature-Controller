#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <max6675.h>
#include <PID_v1.h>
#include <StackArray.h>
#include "src/HeatingSchedule.h"

#define SSR_PIN 9
#define OLED_reset_PIN 7

MAX6675 thermocouple(10, 16, 14); // (SCK pin, CS pin, SO pin)
Adafruit_SSD1306 display(128, 64, &Wire, 7);

uint8_t state = 0;
// PID related parameters
float Setpoint, Input, Output;

float ramp_time_330, ramp_time_445, PID_time_330, PID_time_445;

// I ripped this from our FastDAC code!
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
		  DIRECT);

HeatingSchedule step1(330, 1, 1, 1, 60 * 2);
HeatingSchedule step2(445, 1, 1, 1, 60 * 2);

StackArray<HeatingSchedule> schedule_stack;

void PID_fn(void);
void cool_fn(void);
void set_pid_tune(double, double, double);

void setup()
{
	schedule_stack.push(step2);
	schedule_stack.push(step1);

	Serial.begin(9600);
	pinMode(SSR_PIN, OUTPUT);
	digitalWrite(SSR_PIN, LOW);

	g_pidparam[0].Input = thermocouple.readCelsius();
	myPID.SetMode(AUTOMATIC);
	myPID.SetOutputLimits(0, 255); // although the function defaults to 0 to 255, we call this anyway to be safe

	display.begin(SSD1306_SWITCHCAPVCC, 0x3D);
	delay(100);
	display.setTextSize(1);
	display.setTextColor(SSD1306_WHITE);
	display.display();
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
		cool_fn();
		display.clearDisplay();
		display.setCursor(0, 0);
		display.setTextSize(2);
		display.print("DONE");
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
	while (setpoint - T > 10.0)
	{
		delay(200);
		g_pidparam[0].Input = T;
		myPID.Compute();
		analogWrite(SSR_PIN, g_pidparam[0].Output);
		display.clearDisplay();
		display.setCursor(0, 16);
		display.setTextSize(2);
		display.print(T);
		display.print((char)247);
		display.println("C");
		display.setTextSize(1);
		display.print("Ramp to ");
		display.println(setpoint);
		display.display();
	}

	unsigned long start_time = millis(); // in ms

	// holding
	while ((millis() - start_time) / 1000.0 < ht)
	{
		delay(200);
		g_pidparam[0].Input = T;
		myPID.Compute();
		analogWrite(SSR_PIN, g_pidparam[0].Output);
		display.clearDisplay();
		display.setCursor(0, 16);
		display.setTextSize(2);
		display.print(T);
		display.print((char)247);
		display.println("C");
		display.setTextSize(1);
		display.print("Held for ");
		display.print((millis() - start_time) / 1000.0);
		display.println(" s");
		display.display();
	}
}

void cool_fn(void)
{
	while (thermocouple.readCelsius() > 30)
	{
		delay(200);
		digitalWrite(SSR_PIN, LOW); //SSR Off with HIGH pulse
		display.clearDisplay();
		display.setCursor(0, 0);
		display.setTextSize(2);
		display.println("Cooling");
		display.print(millis() / 1000);
		display.println(" s");
		display.print(thermocouple.readCelsius());
		display.print((char)247);
		display.print("C");
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
