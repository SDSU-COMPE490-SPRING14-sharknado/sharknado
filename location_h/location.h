/*
 * Location.h
 *
 *  Created on: Mar 30, 2014
 *      Author: eggie5
 */

#ifndef LOCATION_H_
#define LOCATION_H_

namespace sharknado {

class Location {
public:
	Location();
	virtual ~Location();
	void computeDistanceAndBearing(double lat1, double lon1,
          double lat2, double lon2, float * results);
	double convertDegMinToDecDeg (float degMin);

};

} /* namespace sharknado */

#endif /* LOCATION_H_ */

