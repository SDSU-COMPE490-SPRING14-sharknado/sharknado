/*
 * BeaconSensor.cpp
 *
 *  Created on: Apr 24, 2014
 *      Author: eggie5
 */
#include <Arduino.h>
#include <iostream>
#include "BeaconSensor.h"
using namespace std;

namespace sharknado {

BeaconSensor::BeaconSensor() {
	// TODO Auto-generated constructor stub

}

int BeaconSensor::sample()
{
	//return RSSI

	//temp hack to fake RSSI reading
	int rssi=random(50,100);

	return rssi;

}

BeaconSensor::~BeaconSensor() {
	// TODO Auto-generated destructor stub
}

} /* namespace sharknado */
