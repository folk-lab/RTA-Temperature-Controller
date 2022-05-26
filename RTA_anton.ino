
#include <SPI.h>
#include <Wire.h>
#include <PID_v1.h>
#include <SparkFunMAX31855k.h>
#include <Adafruit_SSD1306.h>

#include <StackArray.h>
#include "src/HeatingStep.h"

#define SSR_PIN 5
#define START_PIN 20

uint8_t START = LOW;
String x;

SparkFunMAX31855k probe(8);
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

double T;
unsigned long START_TIME;

HeatingStep step0(330, 2.00, 1.2, 0.0, 120, 2.0); // changed delta_t
HeatingStep step1(440, 3.8, 0.9, 0.0, 60, 1.0);  // changed delta_t from 3 to 5
HeatingStep step2(50, 0.0, 0.0, 0.0, 1, 0);

StackArray<HeatingStep> heating_schedule;

void PID_fn(void);
void reset_display(void);
void set_pid_tune(double, double, double);

void setup()
{
    Serial.begin(115200);
    //Serial.setTimeout(1);

    // --------------------------------------------------------------------------------
    heating_schedule.push(step2);
    heating_schedule.push(step1);
    heating_schedule.push(step0);
    // --------------------------------------------------------------------------------

    TCCR1A = 0; //Reset Timer1 control Registor A

    bitClear(TCCR1B, WGM13); //Set CTC mode
    bitSet(TCCR1B, WGM12);

    bitSet(TCCR1B, CS12); //Set prescaler to 1024
    bitClear(TCCR1B, CS11);
    bitSet(TCCR1B, CS10);

    //Reset Timer1
    TCNT1 = 0;

    //set compare value
    //max value  (16bit Timer) = 65535
    /*******************************************
    To calculate compare value
    OCR1A = (time(s) * clock Freq.)/prescaler
    OCR1A = (1*16*10^6)/1024
    ********************************************/
    OCR1A = 1625; //for 0.104 second
    //OCR1A = 3906;     //for 0.25sec
    //OCR1A = 7812;    //for 0.5sec
    //OCR1A = 15625;   //for 1sec
    //OCR1A = 31250;   //for 2sec

    bitSet(TIMSK1, OCIE1A); // Enable Timer1 compare interrupt
    sei();                  // Enable global interrupts

    delay(2000); // do not remove this delay!
    pinMode(SSR_PIN, OUTPUT);
    pinMode(START_PIN, INPUT);
    digitalWrite(SSR_PIN, LOW);

    g_pidparam[0].Input = T;
    myPID.SetSampleTime(50);
    myPID.SetMode(AUTOMATIC);
    myPID.SetOutputLimits(0, 255); // although the function defaults to 0 to 255, we call this anyway to be safe

    display.begin(SSD1306_SWITCHCAPVCC, 0);
    display.setTextColor(SSD1306_WHITE);
    display.setTextSize(2);
    reset_display();
    delay(100);
    display.println("Loading...");
    delay(100);
    display.display();
    delay(2000);
    reset_display();
    delay(100);
}

ISR(TIMER1_COMPA_vect)
{
    T = probe.readTempC();
    if (START == HIGH)
    {
        Serial.print(millis() - START_TIME);
        Serial.print(',');
        Serial.println(T);
    }
}
void loop()
{
  
    if (START == HIGH)
    {
        if (!heating_schedule.isEmpty())
        {
            PID_fn();
            // after ramping, holding
            heating_schedule.pop(); // pop the last entry from the stack
        }
        else
        {
            cli(); // clear interrupt flag. This prevents any interrupts from occuring
            delay(1000);
            analogWrite(SSR_PIN, 0);
            delay(1000);
            START = LOW;
            sei(); // restart interrupts
        }
    }
    else
    {
        delay(110);
        START = digitalRead(START_PIN);
        reset_display();
        display.println("READY");
        display.println(String(T, 2) + String(char(247)) + "C");
        display.display();
        if (START == HIGH)
        {
            cli(); // clear interrupt flag. This prevents any interrupts from occuring
            START_TIME = millis();
            sei(); // restart interrupts
        }
    }

}

void PID_fn(void)
{
    double setpoint = heating_schedule.peek().setpoint;
    double kp = heating_schedule.peek().proportional;
    double ki = heating_schedule.peek().integral;
    double kd = heating_schedule.peek().derivative;
    double ht = heating_schedule.peek().hold_time;
    // temperature difference in ramp
    double delta_t = heating_schedule.peek().delta_t;

    // set the PID parameter
    set_pid_tune(kp, ki, kd);
    g_pidparam[0].Setpoint = setpoint;

    // if the setpoint is delta_t higher, then we fully close the relay
    while ((setpoint - T) >= delta_t)
    {
        delay(110);
        analogWrite(SSR_PIN, 255);
        reset_display();
        display.println(String(T, 2) + String(char(247)) + "C");
        display.println("Ramp to:");
        display.println(String(setpoint, 0) + String(char(247)) + "C");
        display.display();
    }

    // if the overshoot is more than delta_t, then we fully open the relay
    while ((T - setpoint) >= delta_t)
    {
        delay(110);
        analogWrite(SSR_PIN, 0);
        reset_display();
        display.println(String(T, 2) + String(char(247)) + "C");
        display.println("Cool to:");
        display.println(String(setpoint, 0) + String(char(247)) + "C");
        display.display();
    }

    // the temperature is within delta_t of the setpoint, but more than 1 C away from it
    while (abs(setpoint - T) > 1.0)
    {
        delay(110);
        g_pidparam[0].Input = T;
        myPID.Compute();
        analogWrite(SSR_PIN, g_pidparam[0].Output);
        reset_display();
        display.println(String(T, 2) + String(char(247)) + "C");
        display.println("Ramp to:");
        display.println(String(setpoint, 0) + String(char(247)) + "C");
        display.display();
    }

    unsigned long start_time = millis(); // in ms

    // holding
    while ((millis() - start_time) / 1000.0 < ht)
    {
        delay(110);
        g_pidparam[0].Input = T;
        myPID.Compute();
        analogWrite(SSR_PIN, g_pidparam[0].Output);
        reset_display();
        display.println(String(T, 2) + String(char(247)) + "C");
        display.println("Holding");
        display.print((millis() - start_time) / 1000);
        display.println("s");
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


    
//    delay(100);
//    display.clearDisplay();
//    display.setCursor(0, 0);
//    Serial.print("Message " + x + " received");
//    display.println("Message " + x + " received");
//    display.display();




//void loop()
//{
//
//    if (START == HIGH)
//    {
//        if (!heating_schedule.isEmpty())
//        {
//            PID_fn();
//            // after ramping, holding
//            heating_schedule.pop(); // pop the last entry from the stack
//        }
//        else
//        {
//            cli(); // clear interrupt flag. This prevents any interrupts from occuring
//            delay(1000);
//            analogWrite(SSR_PIN, 0);
//            delay(1000);
//            START = LOW;
//            sei(); // restart interrupts
//        }
//    }
//    else
//    {
//        delay(110);
//        START = digitalRead(START_PIN);
//        reset_display();
//        display.println("READY");
//        display.println(String(T, 2) + String(char(247)) + "C");
//        display.display();
//        if (START == HIGH)
//        {
//            cli(); // clear interrupt flag. This prevents any interrupts from occuring
//            START_TIME = millis();
//            sei(); // restart interrupts
//        }
//    }
//}
