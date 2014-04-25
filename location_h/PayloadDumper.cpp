/*
 * PayloadDumper.cpp
 *
 *  Created on: Apr 24, 2014
 *      Author: eggie5
 */

#include "PayloadDumper.h"
#include <iostream>

using namespace std;

namespace sharknado {

PayloadDumper::PayloadDumper() {

    //servo pin attachment
    myservo.attach(8);  // attaches the servo on pin 8 to the servo object
    myservo.write(180); // lock the payload?

}

PayloadDumper::~PayloadDumper() {
	// TODO Auto-generated destructor stub
}

void PayloadDumper::dump()
{
	

#ifdef ARDUINO
           Serial.println("dumping payload w/ servo");
	    int n;
	    myservo.write(0);

	    for (n=0; n<5;n+=1)
	    {
	      myservo.write(180);
	      delay(500);
	      myservo.write(150);
	      delay(500);
	    }
	    myservo.write(0);
	    delay(3000);

	    for (n=0; n<5;n+=1)
	    {
	      myservo.write(30);
	      delay(500);
	      myservo.write(0);
	      delay(500);
	    }

	    myservo.write(180);
   Serial.println("dumping COMPLETE");
#endif

}

} /* namespace sharknado */
