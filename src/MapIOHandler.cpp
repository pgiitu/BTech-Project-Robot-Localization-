/**
 * MapIOHandler.cpp
 *
 *  Created on: 13-Jan-2012
 *      Author: apurv,ashwani,prateek
 */

#include "MapIOHandler.h"
using namespace std;
MapIOHandler::MapIOHandler() {
	// TODO Auto-generated constructor stub

}

MapIOHandler::~MapIOHandler() {
	// TODO Auto-generated destructor stub
}
/**
 * Contructor for MapIOHandler accepting the filename
 */
MapIOHandler::MapIOHandler(const char * filename)
{
	FILE *file = fopen(filename,"r");

	if(file==NULL)
	{
	    cout << "Failed to open file \n";
	}
	else
	{
		int n;
		fscanf(file,"%d",&n);
		Point points[n];
		float x,y;
		for(int i=0;i<n;i++)
		{
			fscanf(file,"%f",&x);
			fscanf(file,"%f",&y);
			points[i]=Point(x,y);
		}
		Polygon pgn(points, points+n);
		map=pgn;
		int m;
		fscanf(file, "%d",&m);
		Point vpoints[m];
		for(int i=0;i<m;i++)
		{
			fscanf(file,"%f",&x);
			fscanf(file,"%f",&y);
			vpoints[i]=Point(x,y);
		}
		Polygon vpgn(vpoints, vpoints+m);
		vPolygon=vpgn;

		fscanf(file,"%f",&x);
		fscanf(file,"%f",&y);
		Point p(x,y);
		robotPos=p;
	}
}

/**
 * Returns the map as a polygon.
 * @return map polygon
 */
Polygon MapIOHandler::GetMapPolygon(){
	return map;
}


/**
 * Returns the visibility polygon of the robot.
 * @return
 */
Polygon MapIOHandler::GetVisibilityPolygon(){
	return vPolygon;
}

/**
 * Returns the Position of the robot
 * @return
 */
Point MapIOHandler::GetRobotPosition(){
	return robotPos;
}
/**
 * Parses the file for map,visibility polygon and robot position
 */
Polygon MapIOHandler::ReturnPolygonFromFile(char *name)
{
	FILE *file = fopen(name,"r");

	if(file==NULL)
	{
	    cout << "Failed to open file \n";
	    Polygon p;
	    return p;
	}
	else
	{
		int n;
		fscanf(file,"%d",&n);
		Point points[n];
		float x,y;
		for(int i=0;i<n;i++)
		{
			fscanf(file,"%f",&x);
			fscanf(file,"%f",&y);
			points[i]=Point(x,y);
		}
		Polygon pgn(points, points+n);
		return pgn;
	}

}

