/*
 * PayloadDumper.h
 *
 *  Created on: Apr 24, 2014
 *      Author: eggie5
 */

#include <iostream>
#include <Arduino.h>
#include <Wire.h>
#include <Servo.h> 


#ifndef PAYLOADDUMPER_H_
#define PAYLOADDUMPER_H_

namespace sharknado {

class PayloadDumper {
public:
	PayloadDumper();
	virtual ~PayloadDumper();
	//Sevo Code
        Servo myservo;  // create servo object to control a servo 
                // twelve servo objects can be created on most boards


	void dump();
};

} /* namespace sharknado */

#endif /* PAYLOADDUMPER_H_ */
