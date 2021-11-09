#include <PID_v1.h>
#include "Timer.h"

#define sensorPin 2
#define motorPin 9
#define dents 40.0
#define maxSpeed 170
#define startRampTime 5
#define waitTime 3

//Functions
void calculateRPM();
void getDelta();
void sendData();

//Variables
double setpoint = 0, rpm = 0, output, calc, x, y, m, b, alfa = 0.3;
unsigned long tempo_anterior = 0, delta = 0;
int state = 1, tempo = 0;
String inputString = "", outputString = "";     // A String to hold incoming data
bool flag = true, stringComplete = false; // Whether the string is complete

//Timer
Timer t;

//Specify PID links and initial tuning parameters
double Kp = 6, Ki = 4.5, Kd = 0.1;
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
    //t.every(500, sendData, 0);
    //turn the PID on
    myPID.SetMode(AUTOMATIC);
}

void loop()
{
    //Update Timer
    t.update();
  
    //State 1
    if (state == 1 && ((millis() * 0.001) - tempo) >= waitTime)
    {
        //Calculate b
        if (flag)
        {
            x = millis() * 0.001;
            y = setpoint;
            m = maxSpeed / startRampTime;
            b = y - m * x;
            flag = false;
        }
        //acelarate to top speed in 5 seconds
        if (setpoint < maxSpeed)
            setpoint = m * (millis() * 0.001) + b;
        //Set top speed and move to next state
        else
        {
            setpoint = maxSpeed;
            tempo = millis() * 0.001;
            state = 2;
            flag = true;
        }
    }
    //State 2
    else if (state == 2 && ((millis() * 0.001) - tempo) >= waitTime)
    {
        //Set speed to 50%
        setpoint = maxSpeed * 0.5;
        tempo = millis() * 0.001;
        state = 3;
    }
    //State 3
    else if (state == 3 && ((millis() * 0.001) - tempo) >= waitTime)
    {
        //Set speed to 75%
        setpoint = maxSpeed * 0.75;
        tempo = millis() * 0.001;
        state = 4;
    }
    //State 4
    else if (state == 4 && ((millis() * 0.001) - tempo) >= waitTime)
    {
        //Set speed to 25%
        setpoint = maxSpeed * 0.25;
        tempo = millis() * 0.001;
        state = 5;
    }
    //State 5
    else if (state == 5 && ((millis() * 0.001) - tempo) >= waitTime)
        //Set speed to 0
        setpoint = 0;

    //Get rpm value
    calculateRPM();

    //Compute PID
    myPID.Compute();
    //Generate PWM
    analogWrite(motorPin, output);

    
    //Print setpoint
    Serial.print("Setpoint: ");
    Serial.print(setpoint);
    //Print rpm
    Serial.print(" rpm: ");
    Serial.print(rpm);
    //Print output
    Serial.print(" output: ");
    Serial.println(output);
    
    /*
    // Print the string when a newline arrives:
    if (stringComplete)
    {
        Serial.println(inputString);
        // clear the string
        inputString = "";
        stringComplete = false;
    }
    */
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
