/*
 * BeaconSensor.h
 *
 *  Created on: Apr 24, 2014
 *      Author: eggie5
 */

#ifndef BEACONSENSOR_H_
#define BEACONSENSOR_H_

namespace sharknado {

class BeaconSensor {
public:
	BeaconSensor();
	virtual ~BeaconSensor();

	int sample();
};

} /* namespace sharknado */

#endif /* BEACONSENSOR_H_ */
