#include <PID_v1.h>
#include "Timer.h"

#define sensorPin 2
#define motorPin 9
#define dents 40.0
#define startRampTime 10
#define stopRampTime 5

//Functions
void calculateRPM();
void getDelta();
void sendData();

//Variables
double setpoint = 0, rpm = 0, output, calc, x, y, m, b, limit, alfa = 0.3;
unsigned long tempo_anterior = 0, delta = 0;
int state = 1;
String inputString = "", outputString = "";     // A String to hold incoming data
bool flag = true, stringComplete = false; // Whether the string is complete

//Timer
Timer t;

//Specify PID links and initial tuning parameters
double Kp = 6,Ki = 4, Kd = 0.3;
PID myPID(&rpm, &output, &setpoint, Kp, Ki, Kd, DIRECT);

void setup()
{
    //Initialize serial
    Serial.begin(9600);
    // Reserve 200 bytes for the inputString:
    inputString.reserve(200);

    //Initialize pins
    pinMode(sensorPin, INPUT);
    pinMode(motorPin, OUTPUT);
    attachInterrupt(digitalPinToInterrupt(sensorPin), getDelta, RISING);

    //Send data through serial
    t.every(100, sendData, 0);
    //turn the PID on
    myPID.SetMode(AUTOMATIC);
}

void loop()
{
    //Update timer
    t.update();
    
    //Start motor
    if (state == 1 && limit > 0)
    {
        //Calculate b
        if (flag)
        {
            x = millis() * 0.001;
            y = setpoint;
            m = limit / startRampTime;
            b = y - m * x;
            flag = false;
        }
        
        //Start ramp
        setpoint = m * (millis() * 0.001) + b;
        if(setpoint >= limit)
        {
          setpoint = limit;
          state = 2; 
          flag = true; 
        }
    }
    //Stop motor
    if (state == 2 && limit == 0)
    {
        //Calculate b
        if (flag)
        {
            x = millis() * 0.001;
            y = setpoint;
            m = setpoint / stopRampTime * -1;
            b = y - m * x;
            flag = false;
        }
        
        //Stop ramp
        setpoint = m * (millis() * 0.001) + b ;
        if(setpoint <= 0)
        {
          setpoint = 0;
          state = 1;
          flag = true;
        }  
    }

    //Compute PID
    myPID.Compute();
    //Generate PWM
    analogWrite(motorPin, output);  
    
    //get rpm
    calculateRPM();      
    
    // Print the string when a newline arrives:
    if (stringComplete)
    {
        Serial.println(inputString);
        // clear the string
        limit = inputString.toDouble();
        Serial.println(limit);
        inputString = "";
        stringComplete = false;
    }
}


void calculateRPM()
{
  if (delta > 0)
    calc = 60.0 / ((delta * 0.000001) * dents);
  else
    calc = 0;
    
  if(calc < 300)
    rpm = alfa * calc + (1 - alfa) * rpm;
    
  if((micros() - tempo_anterior) > 1000000)
    rpm = 0;
}

void getDelta()
{
    noInterrupts();
    delay(0.3);
    if(digitalRead(sensorPin) == HIGH)
    {
      delay(0.3);
      if(digitalRead(sensorPin) == HIGH)
      {
            delta = micros() - tempo_anterior;
            tempo_anterior = micros();
      }
    }
    interrupts();
}

void serialEvent()
{
  while (Serial.available())
  {
    // get the new byte:
    char inChar = (char)Serial.read();
    // add it to the inputString:
    inputString += inChar;
    // if the incoming character is a newline, set a flag so the main loop can
    // do something about it:
    if (inChar == '\n')
      stringComplete = true;
  }
}

 void sendData()
{
    outputString = "rpm = " + String(rpm, 2);
    Serial.println(outputString);
    outputString = "Setpoint: " + String(setpoint);
    Serial.println(outputString);
}
