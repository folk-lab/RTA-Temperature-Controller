#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h> // need install
#include <Adafruit_SSD1306.h> // need install
#include <max6675.h> // need install
#include <PID_v1.h> // need install

#define SSR_PIN 9
#define OLED_reset_PIN 7

MAX6675 thermocouple(10, 16, 14); // (SCK pin, CS pin, SO pin)
Adafruit_SSD1306 display(128, 64, &Wire, 7);

uint8_t state = 0;
// PID related parameters
float Setpoint, Input, Output;

float T; 
float ramp_time_330, ramp_time_445, PID_time_330, PID_time_445;

// I ripped this from our FastDAC code!
typedef struct PIDparam
{
	bool forward_dir = true;
	double Input = 0.0;
	double Output = 0.0;
	double Setpoint = 0.0;
	double kp = 0.9;
	double ki = 1;
	double kd = 0.1;
} PIDparam;

PIDparam g_pidparam[1];

PID myPID(&(g_pidparam[0].Input),
		  &(g_pidparam[0].Output),
		  &(g_pidparam[0].Setpoint),
		  g_pidparam[0].kp,
		  g_pidparam[0].ki,
		  g_pidparam[0].kd,
		  DIRECT);

void PID_fn(void);
void cool_fn(void);
void set_pid_tune(double, double, double); // to set the tuning parameters

void setup()
{
	Serial.begin(9600);
	pinMode(SSR_PIN, OUTPUT);
	digitalWrite(SSR_PIN, LOW);
	
	Input = thermocouple.readCelsius();
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
	if (state == 0)
	{
		PID_fn();
	}
	else if (state == 1)
	{
		cool_fn();
	}
	else if (state == 2)
	{
		display.clearDisplay();
		display.setCursor(0, 0);
		display.setTextSize(2);
		display.print("DONE");
		display.display();
	}
	else
	{
		cool_fn();
	}
}

void PID_fn(void)
{
	delay(200);
	T = thermocouple.readCelsius();
	g_pidparam[0].Input = T;

	myPID.Compute();
	analogWrite(SSR_PIN, g_pidparam[0].Output);
	display.clearDisplay();
	display.setCursor(0, 16);
	display.setTextSize(2);
	display.print(T);
	display.print((char)247);
	display.println("C");

	if (T < 320 && g_pidparam[0].Setpoint == 330)
	{
		ramp_time_330 = millis();
		display.setTextSize(1);
		display.println("Ramp to 330");
		display.display();
	}

	else if (T > 320 && g_pidparam[0].Setpoint == 330)
	{
		PID_time_330 = millis() - ramp_time_330;
		display.setTextSize(1);
		display.println("Hold at 330 for 2 min");
		display.print(PID_time_330 / 1000);
		display.println(" s");
		display.display();

		if (PID_time_330 > 120000)
		{
			g_pidparam[0].Setpoint = 445;
			set_pid_tune(1, 1, 1);
		}
	}

	else if (T < 435 && g_pidparam[0].Setpoint == 445)
	{
		ramp_time_445 = millis();
		display.setTextSize(1);
		display.println("Ramp to 445");
		display.display();
	}

	else if (T > 435 && g_pidparam[0].Setpoint == 445)
	{
		PID_time_445 = millis() - ramp_time_445;
		display.setTextSize(1);
		display.println("Hold at 445 for 2 min");
		display.print(PID_time_445 / 1000);
		display.println(" s");
		display.display();
		if (PID_time_445 > 120000)
		{
			state = 1;
		}
		else
		{
		}
	}
}

void cool_fn(void)
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
	if (thermocouple.readCelsius() < 30)
	{
		state = 2;
	}
}

void set_pid_tune(double kp, double ki, double kd)
{
	g_pidparam[0].kp = kp;
	g_pidparam[0].ki = ki;
	g_pidparam[0].kd = kd;
	myPID.SetTunings(g_pidparam[0].kp, g_pidparam[0].ki, g_pidparam[0].kd);
}