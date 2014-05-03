/*
 * ProximitySensor.h
 *
 *  Created on: Apr 24, 2014
 *      Author: eggie5
 */

#ifndef PROXIMITYSENSOR_H_
#define PROXIMITYSENSOR_H_

namespace sharknado {
   int trig_pin;
  static int echo_pin;
  static volatile long csample;
class ProximitySensor {
public:
	ProximitySensor(int trigpin, int echopin) ;
	virtual ~ProximitySensor();
        static void timer_callback();
        int sample();
        int getEchoPin();
        
};

} /* namespace sharknado */

#endif /* PROXIMITYSENSOR_H_ */
