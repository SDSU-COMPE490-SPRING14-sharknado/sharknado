/*
 * ProximitySensor.cpp
 *
 *  Created on: Apr 24, 2014
 *      Author: eggie5
 */

#include "ProximitySensor.h"
#include <DueTimer.h>
#include <Arduino.h>

namespace sharknado {
  


//probably attach to a timer interrupt
//run the sensor every T and invoke
//a callback on the main loop

ProximitySensor::ProximitySensor(int echopin, int trigpin) {
  echo_pin=echopin;
  trig_pin=trigpin;
  
  pinMode(trig_pin, OUTPUT);
  pinMode(echo_pin, INPUT);
  
  
 Timer.getAvailable().attachInterrupt(this->timer_callback).start(500000);
 
}

int ProximitySensor::sample()
{
  long distance=(csample/2) / 29.1;
  if (distance >= 200 || distance <= 0)
    return -1; //clear
  else
    return distance; //cm
    
}

int ProximitySensor::getEchoPin()
{
  return echo_pin;
}



 void ProximitySensor::timer_callback() 
{
  digitalWrite(trig_pin, LOW);  
  delayMicroseconds(2);  
  digitalWrite(trig_pin, HIGH);
  delayMicroseconds(10);  
  digitalWrite(trig_pin, LOW);
  csample = pulseIn(echo_pin, HIGH);
}

ProximitySensor::~ProximitySensor() {
	// TODO Auto-generated destructor stub
}

} /* namespace sharknado */
