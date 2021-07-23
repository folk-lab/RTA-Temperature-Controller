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
double Setpoint, Input, Output, kp, ki, kd;
float T;
float ramp_time_330, ramp_time_445, PID_time_330, PID_time_445;

PID myPID(&Input, &Output, &Setpoint, &kp, &ki, &kd, DIRECT);

void setup() {
  Serial.begin(9600);
  pinMode(SSR_PIN, OUTPUT);
  digitalWrite(SSR_PIN, LOW);

  Setpoint = 330;
  kp = 1;
  ki = 1;
  kd = 1;
  Input = thermocouple.readCelsius();
  myPID.SetMode(AUTOMATIC);
  
  display.begin(SSD1306_SWITCHCAPVCC, 0x3D);
  delay(100);
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.display();
  delay(100);
}

void loop() {
  if (state==0){
    PID_fn();
  }
  else if (state==1){
    cool_fn();
  }
  else if (state==2){
    display.clearDisplay();
    display.setCursor(0, 0);
    display.setTextSize(2);
    display.setTextColor(SSD1306_WHITE);
    display.print("DONE");
  }
}

void PID_fn(void){   
  delay(200); 
  T = thermocouple.readCelsius(); 
  Input = T;
  myPID.Compute();
  analogWrite(SSR_PIN, Output);
  display.clearDisplay();
  display.setCursor(0, 16);
  display.setTextSize(2);
  display.setTextColor(SSD1306_WHITE);
  display.print(T);
  display.print((char)247);
  display.println("C");

  if (T < 320 && Setpoint==330){
    ramp_time_330 = millis();
    display.setTextSize(1);
    display.println("Ramp to 330");
    display.display();
  }

  else if (T > 320 && Setpoint==330){
    PID_time_330 = millis() - ramp_time_330;
    display.setTextSize(1);
    display.println("Hold at 330 for 2 min");
    display.print(PID_time_330/1000);
    display.println(" s");
    display.display();
    
    if (PID_time_330 > 120000){
      Setpoint = 445;
      kp = 1;
      ki = 1;
      kd = 1;
    }
  }
    
  else if (T < 435 && Setpoint==445){
    ramp_time_445 = millis();
    display.setTextSize(1);
    display.println("Ramp to 445");
    display.display();
  }

  else if (T > 435 && Setpoint==445){
    PID_time_445 = millis() - ramp_time_445;
    display.setTextSize(1);
    display.println("Hold at 445 for 2 min");
    display.print(PID_time_445/1000);
    display.println(" s");
    display.display();
    if (PID_time_445 > 120000){
      state = 1;
    }
    else{
      
    }
    
  }
}

void cool_fn(void){
  delay(200);
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
  if (thermocouple.readCelsius()<30){
    state=2;
  }
}




    
