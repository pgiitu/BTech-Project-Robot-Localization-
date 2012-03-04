/*
 * MapIOHandler.h
 *
 *  Created on: 13-Jan-2012
 *      Author: apurv
 */
#include "constants.h"

#ifndef MAPIOHANDLER_H_
#define MAPIOHANDLER_H_

class MapIOHandler {
public:

	Polygon map;
	Polygon vPolygon;
	Point robotPos;

	MapIOHandler();
	MapIOHandler(const char *);
	virtual ~MapIOHandler();


	Polygon GetMapPolygon();
	Polygon GetVisibilityPolygon();
	Polygon ReturnPolygonFromFile(char *name);
	Point GetRobotPosition();

};

#endif /* MAPIOHANDLER_H_ */
