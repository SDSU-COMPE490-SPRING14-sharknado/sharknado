#include "FastRunningMedian.h"

int sensorPin = A0;    // select the input pin for the potentiometer
int sensorValue = 0;  // variable to store the value coming from the sensor
FastRunningMedian<unsigned int,7, 0> median;

void setup() {
  // declare the ledPin as an OUTPUT:
  pinMode(sensorPin, INPUT);
  
  Serial.begin(9600);
    Serial.println("antenna demo");
  
}

unsigned int max_reading=0;
int stop_count=0;
void loop() {
  // read the value from the sensor:
  sensorValue = analogRead(sensorPin);
  median.addValue(sensorValue);
  
  unsigned int sample = median.getMedian();
  Serial.println( sample);
  
  if(sample >= max_reading)
  {
    max_reading = sample;
    stop_count=0;
    
    Serial.println(" \t keep going");
  }
  else
  {
    stop_count++;
    if( sample>25)
      Serial.println(" \t stop searching");
  }
  

    delay(100);
}

//beacon off, no motors stable around 9
//beacon onn, no motors max distance ~ 1.8 mters
//2 feet = 50
//1 foot = 256
