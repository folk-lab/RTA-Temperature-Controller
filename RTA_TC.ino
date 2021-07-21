/* Arduino Pro Micro Code for RTA Temperature Controller */

#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "max6675.h"
#include <PID_v1.h>

#define OLED_reset_PIN 7
Adafruit_SSD1306 display(128, 64, &Wire, 7);

MAX6675 thermocouple(10, 16, 14);    // (SCK pin, CS pin, SO pin)

#define SSR_PIN  9

uint8_t state = 0;
double Setpoint, Input, Output;
float kp = 1;
float ki = 0.5;
float kd = 0.5;

PID myPID(&Input, &Output, &Setpoint, kp, ki, kd, DIRECT);

void setup() {
  Serial.begin(9600);
  pinMode(SSR_PIN, OUTPUT);
  digitalWrite(SSR_PIN, LOW);

  Setpoint = 330;
  Input = thermocouple.readCelsius();
  myPID.SetMode(AUTOMATIC);
  //myPID.SetSampleTime(300);
  
  display.begin(SSD1306_SWITCHCAPVCC, 0x3D);
  delay(100);
  display.setTextSize(2);
  display.setTextColor(SSD1306_WHITE);
  display.display();
  delay(100);
}


void loop() {
  if(state == 0){
    ramp_fn(); 
  }
  else if (state == 1){
    PID_fn(); 
  }
  else{
    cool_fn(); 
  }
}

void ramp_fn(void){
  delay(200); 
  display.clearDisplay();
  display.setCursor(0, 0);
  display.setTextSize(2);
  display.setTextColor(SSD1306_WHITE);  
  display.println("Ramping");
  display.print(millis()/1000);
  display.println(" s");
  display.print(thermocouple.readCelsius());
  display.print((char)247);
  display.print("C");
  display.display();
  
  if(thermocouple.readCelsius() < Setpoint){
    digitalWrite(SSR_PIN, HIGH);                // SSR On  
  }
  else{
    digitalWrite(SSR_PIN, LOW);                 // SSR Off
    state = 1;
  }
}

void PID_fn(void){   
  delay(200);    
  Input = thermocouple.readCelsius();
  myPID.Compute();
  analogWrite(SSR_PIN, Output);
  display.clearDisplay();
  display.setCursor(0, 0);
  display.setTextSize(2);
  display.setTextColor(SSD1306_WHITE);  
  display.println("PID");
  display.print(millis()/1000);
  display.println(" s");
  display.print(thermocouple.readCelsius());
  display.print((char)247);
  display.print("C");
  display.display();
}

void cool_fn(void){
  digitalWrite(SSR_PIN, LOW);    //SSR Off with HIGH pulse
  display.clearDisplay();
  display.setCursor(0, 0);
  display.setTextSize(2);
  display.setTextColor(SSD1306_WHITE);  
  display.println("Cooling");
  display.print(millis()/1000);
  display.println(" s");
  display.print(thermocouple.readCelsius());
  display.print((char)247);
  display.print("C");
  display.display();
}
    
