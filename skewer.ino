/*
A library for computer controlled camera panning, specifically for 
skateboard motion blur (ie. "skater blur" -> "skewer")

Copyright 2015 Micah Wedemeyer
*/

#include <SPI.h>
#include <Pixy.h>
#include <Servo.h>

Pixy pixy;
Servo servo;

const long servoPin = 9;
const long pixyViewAngle = 75;

const long shutterPin = 2;
const boolean takePicture = false;

const long servoLow = 0;
const long servoHigh = 180;
const long servoStart = 45;
const long pixyLow = PIXY_MIN_X;
const long pixyHigh = PIXY_MAX_X;
const long pixyCenter = pixyHigh / 2;
const long centerBoxWidth = 0; // The number of pixels that are allowed to be the center
const boolean reversed = false;
const boolean debug = false;

const long kp = 20; // Kp - proportial term for PID
const long ki = 1; // Ki - integral term for PID
const long kd = 140; // Kd - derivative term for PID
const long gainFactor = 10; // Stolen from PanTilt -> bitshift the entire PID result to get something the servo can really understand

long servoPos;    // variable to store the servo position 
long pixyX = pixyLow;
boolean acquired = false;
long errorXSum = 0;
const long errorXNull = 99999;
long errorXPrev = errorXNull;

unsigned long lastPositionUpdate = 0;
unsigned long positionUpdateMs = 20;
 
void setup() 
{ 
  Serial.begin(9600);
  
  pixy.init();
  servo.attach(servoPin);
  
  servoPos = servoStart;
  servo.write(servoPos);
  
  pinMode(shutterPin, OUTPUT);
  digitalWrite(shutterPin, LOW);
} 

long calcError(long sp, long pv) {
  long error;
  error = sp - pv;
  
  if(abs(error) < centerBoxWidth) {
    error = 0;
  }
  
  return error;
}

long calcPidVelocity(long error, long errorSum, long prevError) {
  long pVal;
  long iVal;
  long dVal;
  long vel;
  
  pVal = kp * error;
  iVal = ki * (error + errorSum);
  
  if(debug) {
    Serial.print("Error: ");
    Serial.println(error);
    Serial.print("prevError: ");
    Serial.println(prevError);
  }
  
  if(prevError == errorXNull) {
    prevError = error;
  }
  
  dVal = kd * (error - prevError);
  
  vel = (pVal + iVal + dVal) >> gainFactor;
  
  if(debug) {
    Serial.print("pVal: ");
    Serial.println(pVal);
    Serial.print("iVal: ");
    Serial.println(iVal);
    Serial.print("dVal: ");
    Serial.println(dVal);  
    Serial.print("Vel: ");
    Serial.println(vel);
    Serial.println("");
  }
  
  return vel;
}


void loop() 
{ 
  uint16_t blocks;
  long newPos = 0;
  long vel;
  long errorX;
  
  blocks = pixy.getBlocks();
  if(blocks) {
    acquired = true;
  } else {
    acquired = false;
    errorXSum = 0;
    errorXPrev = errorXNull;
  }
  
  if (acquired) { 
    if(takePicture) {
      digitalWrite(shutterPin, HIGH);
    }
    
    pixyX = pixy.blocks[0].x;
    errorX = calcError(pixyCenter, pixyX);
    errorXSum += errorX;
    vel = calcPidVelocity(errorX, errorXSum, errorXPrev);
    errorXPrev = errorX;
    
    if(reversed) {
      vel *= -1;
    }
    
    newPos = servoPos + vel;
    newPos = max(min(newPos, servoHigh), servoLow);

    if(abs(vel) > 0) {
      servo.write(newPos);
    }

    // We don't actually know where the servo is, so we don't update our position assumption of it every time.
    // It's like a delay, but without completely blocking everything else
    if(millis() - lastPositionUpdate > positionUpdateMs) {
      servoPos = newPos;
      lastPositionUpdate = millis();
    }
  } else {
    digitalWrite(shutterPin, LOW);
  }
} 

