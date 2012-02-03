/*
 * MapIOHandler.cpp
 *
 *  Created on: 13-Jan-2012
 *      Author: apurv
 */

#include "MapIOHandler.h"

MapIOHandler::MapIOHandler() {
	// TODO Auto-generated constructor stub

}

MapIOHandler::~MapIOHandler() {
	// TODO Auto-generated destructor stub
}

/**
 * Returns the map as a polygon.
 * @return
 */
Polygon MapIOHandler::GetMapPolygon(){

	//	Point points[] = { Point(0,0), Point(5.1,0), Point(1,1), Point(0.5,6)};
	//	Point points[]={Point(-1,-1),Point(-1,-2),Point(1,-2),Point(0,0),Point(1,0),Point(0,1),Point(-1,0),Point(0,-1)};
	//	Point points[]={Point(1.0,-2.0),Point(1.0,0.0),Point(0.0,1.0),Point(-1.0,0.0),Point(0.0,0.0),Point(0.0,-2.0)};
/*
	Point points[]={Point(0,-1),Point(10,-1),Point(10,3),Point(11,3),Point(11,4),
					Point(9,4),Point(9,0),Point(6,0),Point(6,8),Point(7,8),Point(7,9),
					Point(5,9),Point(5,0),Point(1,0),Point(1,4),Point(2,4),Point(2,5),
					Point(0,5)
				   };
	Polygon pgn(points, points+18);
*/

	//	Point points[]={Point(-1,-1),Point(1,-1)};

		Point points[]={Point(0,0),Point(4,0),Point(4,4),Point(0,4)};
		Polygon pgn(points, points+4);

		return pgn;
}


/**
 * Returns the visibility polygon of the robot.
 * @return
 */
Polygon MapIOHandler::GetVisibilityPolygon(){

	Point points[] = {Point(5,7),Point(6,8),Point(7,8),Point(7,9),Point(5,9)};
	Polygon pgn(points, points+5);

	return pgn;
}


