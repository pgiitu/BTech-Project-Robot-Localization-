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
	MapIOHandler();
	virtual ~MapIOHandler();

	Polygon GetMapPolygon();
	Polygon GetVisibilityPolygon();

};

#endif /* MAPIOHANDLER_H_ */
