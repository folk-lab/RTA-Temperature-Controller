/*
  QDEV UBC, 2021
  Ray: raysu@student.ubc.ca; github @ sillyPhotons
  Anton: antoncecic3@gmail.com
  
  Timer Interrupt Code From: https://www.techtonions.com/6-simple-ways-to-blink-arduino-led/
*/

#include <SPI.h>
#include <Wire.h>


// libraries that likely need to be downloaded in library manager
#include <PID_v1.h>
#include <SparkFunMAX31855k.h>
#include <Adafruit_SSD1306.h>

// additional libraries included with the file
#include <StackArray.h>
#include "src/HeatingStep.h"

#define SSR_PIN 5         // PWM pin that activates the relay
#define START_PIN 23      // Pin that is hooked up to the START button

uint8_t START = LOW;

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
unsigned long LastSerialSend = 0;

// --------------------------------------------------------------------------------
// Temperature [C], kP, kI, kD, Seconds to Hold Temperature At, delta_t

// Set knob to 60% full power
// Ebrahim's Sequence
//HeatingStep step0(300, 4.00, 1.2, 0.0, 3600, 6.0);
//HeatingStep step1(50, 0.0, 0.0, 0.0, 1, 0);

// Ray's Sequence
//HeatingStep step0(330, 4.00, 1.2, 0.0, 120, 6.0); // changed delta_t
//HeatingStep step1(445, 3.8, 0.9, 0.0, 120, 5.0);  // changed delta_t from 3 to 5
//HeatingStep step2(50, 0.0, 0.0, 0.0, 1, 0);

//HeatingStep step0(330, 2.00, 1.2, 0.0, 120, 6.0); // changed delta_t
//HeatingStep step1(445, 3.8, 0.9, 0.0, 120, 5.0);  // changed delta_t from 3 to 5
//HeatingStep step2(50, 0.0, 0.0, 0.0, 1, 0);

String sequence_name = "Anton's seq.";
HeatingStep step0(450, 2.00, 1.2, 0.0, 60*60*5, 7.0); // changed delta_t
HeatingStep step1(50, 0.0, 0.0, 0.0, 1, 0);

// Set knob to 70% full power
// HeatingStep step1(450, 3.8, 0.9, 0.0, 60, 5.0); // changed delta_t from 3 to 5

// 300 C 5 mbar anneal for 10 minutes
// Set knob to 35% full power      
// HeatingStep step0(300, 4.00, 1.0, 0.0, 600, 1.0);
// HeatingStep step1(50, 0.0, 0.0, 0.0, 1, 0);
// --------------------------------------------------------------------------------

StackArray<HeatingStep> heating_schedule;

void PID_fn(void);
void reset_display(void);
void set_pid_tune(double, double, double);

void setup()
{
    Serial.begin(19200);

    // --------------------------------------------------------------------------------
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
    
    reset_display();
    delay(100);

    display.drawRect(4, 2, 120, 60, WHITE);
    
    display.setTextSize(3);
    display.setCursor(8, 8);
    display.print("RTA");
    
    display.setTextSize(1);
    display.setCursor(8, 35);
    display.print("Folklab Instruments");
    display.display();
    delay(2000);
    reset_display();
    delay(100);
}

ISR(TIMER1_COMPA_vect)
{
    T = probe.readTempC();
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
        display.setTextSize(2);
        display.setCursor(0, 0);
        display.print("Push START");

        display.setTextSize(1);
        display.setCursor(0, 15);
        display.print("---------------------");

        display.setTextSize(1);
        display.setCursor(0, 20);
        display.print("Loaded sequence:");
        display.setCursor(0, 28);
        display.print(">> " + sequence_name);
        display.setCursor(0, 35);
        display.print("---------------------");

   

        display.setTextSize(2);
        display.setCursor(0, 45);
        display.println("T:" + String(T, 2) + String(char(247)) + "C");
        display.println();
        display.display();

        serial_fn();
        
        if (START == HIGH)
        {
            cli(); // clear interrupt flag. This prevents any interrupts from occuring
            START_TIME = millis();
            sei(); // restart interrupts
        }
    }
}

void serial_fn(void)
{
  if (millis() - LastSerialSend > 50)
    {
    LastSerialSend = millis();
    Serial.print(millis());
    Serial.print(',');
    Serial.println(T);
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

        serial_fn();
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

        serial_fn();
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

        serial_fn();
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

        serial_fn();
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
