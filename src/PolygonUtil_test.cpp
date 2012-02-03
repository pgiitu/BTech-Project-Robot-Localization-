/*
 * PolygonUtil_test.cpp
 *
 *  Created on: 13-Jan-2012
 *      Author: apurv
 */

#include "PolygonUtil.h"

void VisiblePointSetTest(){

	PolygonUtil pUtil = PolygonUtil();

	Point points[]={Point(-1,-1),Point(-1,-2),Point(1,-2),Point(0,0),Point(1,0),Point(0,1),Point(-1,0),Point(0,-1)};
	Polygon map(points, points+8);

	Point robotPos = Point(0.1, -1.8);

	Polygon vPointSet = pUtil.VisiblePointSet(map, robotPos);

	pUtil.DisplayPolygon(vPointSet);
}
