#include <SPI.h>
#include <Wire.h>

// libraries that likely need to be downloaded in library manager
#include <PID_v1.h>
#include <SparkFunMAX31855k.h> // Using the max31855k driver
//#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// additional libraries included with the file
#include <StackArray.h>
#include "src/Bitmap.h"
#include "src/HeatingStep.h"

#define SSR_PIN 10
#define START_PIN 20

uint8_t START = LOW;

SparkFunMAX31855k probe(10);
Adafruit_SSD1306 display(128, 64, &Wire, -1);

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
HeatingStep step0(330, 4.7, 0.9, 0.0, 1);
HeatingStep step1(330, 4.00, 1.2, 0.0, 120); // Set knob to 60% full power
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
	/*
	Modify below 
	*/
	heating_schedule.push(step3);
	heating_schedule.push(step2);
	heating_schedule.push(step1);
	heating_schedule.push(step0);
	/*
	Modify above
	*/

	delay(2000); // do not remove this delay!
	pinMode(SSR_PIN, OUTPUT);
	pinMode(START_PIN, INPUT);
	digitalWrite(SSR_PIN, LOW);

	g_pidparam[0].Input = probe.readTempC();
	myPID.SetMode(AUTOMATIC);
	myPID.SetOutputLimits(0, 255); // although the function defaults to 0 to 255, we call this anyway to be safe

	display.begin(SSD1306_SWITCHCAPVCC, 0);
	display.setTextColor(SSD1306_WHITE);
	display.setTextSize(2);
	reset_display();
	delay(100);
	display.drawBitmap(0, 0, myBitmap, 125, 65, WHITE);
	delay(100);
	display.display();
	delay(2000);
	reset_display();
	delay(100);
}

void loop()
{
	if (START == HIGH){
		if (!heating_schedule.isEmpty())
		{
			PID_fn();
			// after ramping, holding
			heating_schedule.pop(); // pop the last entry from the stack
		}
		else
		{
			reset_display();
			display.println("COMPLETE");
			display.display();
		}		
	}
	else{
		delay(200);
		START = digitalRead(START_PIN);
		reset_display();
		display.println("READY");
		display.println(String(probe.readTempC(), 2) + String(char(247)) + "C");
		display.display();
	}
}

void PID_fn(void)
{
	double setpoint = heating_schedule.peek().setpoint;
	double kp = heating_schedule.peek().proportional;
	double ki = heating_schedule.peek().integral;
	double kd = heating_schedule.peek().derivative;
	double ht = heating_schedule.peek().hold_time;
	double T = thermocouple.readCelsius();

	set_pid_tune(kp, ki, kd);
	g_pidparam[0].Setpoint = setpoint;

	// ramping sequence
	while (abs(setpoint - T) > 1.0)
	{
		delay(200);
		T = probe.readTempC();
		g_pidparam[0].Input = T;
		myPID.Compute();
		analogWrite(SSR_PIN, g_pidparam[0].Output);
		reset_display();
		display.println(String(T, 2)+ String(char(247)) + "C");
		display.println("Ramp " + String(setpoint, 0) + String(char(247)) + "C");
		display.display();
	}

	unsigned long start_time = millis(); // in ms

	// holding
	while ((millis() - start_time) / 1000.0 < ht)
	{
		delay(200);
		T = probe.readTempC();
		g_pidparam[0].Input = T;
		myPID.Compute();
		analogWrite(SSR_PIN, g_pidparam[0].Output);
		reset_display();
		display.println(String(T, 2) + String(char(247)) + "C");
		display.println("Holding " + String((millis() - start_time) / 1000, 1) + "s");
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
	display.setCursor(0, 0);
}

void append_to_display(String message)
{
	display.println(message);
}
