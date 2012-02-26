/*
 * MapIOHandler.cpp
 *
 *  Created on: 13-Jan-2012
 *      Author: apurv
 */

#include "MapIOHandler.h"
using namespace std;
MapIOHandler::MapIOHandler() {
	// TODO Auto-generated constructor stub

}

MapIOHandler::~MapIOHandler() {
	// TODO Auto-generated destructor stub
}

MapIOHandler::MapIOHandler(char * filename)
{
	FILE *file = fopen(filename,"r");

	if(file==NULL)
	{
	    cout << "Failed to open file \n";
/*
	    map=NULL;
	    vPolygon=NULL;
	    robotPos=NULL;
*/
	}
	else
	{
		int n;
		fscanf(file,"%d",&n);
		cout <<"the n is "<<n;
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
/*
		robotPos.cartesian(0)=x;
		robotPos.cartesian(1)=y;
*/

	}

}

/**
 * Returns the map as a polygon.
 * @return
 */
Polygon MapIOHandler::GetMapPolygon(){
	//	Point points[] = { Point(0,0), Point(5.1,0), Point(1,1), Point(0.5,6)};
/*
		Point points[]={Point(-1,-1),Point(-1,-2),Point(1,-2),Point(0,0),Point(1,0),Point(0,1),Point(-1,0),Point(0,-1)};
		Polygon pgn(points, points+8);
		return pgn;
*/
	//	Point points[]={Point(1.0,-2.0),Point(1.0,0.0),Point(0.0,1.0),Point(-1.0,0.0),Point(0.0,0.0),Point(0.0,-2.0)};

/*
	Point points[]={
			Point(0,-1),Point(11,-1),Point(11,12),Point(12,12),
			Point(12,13),Point(10,13),Point(10,0),Point(6,0),
			Point(6,8),Point(7,8),Point(7,9),
			Point(5,9),Point(5,0),Point(1,0),Point(1,4),Point(2,4),Point(2,5),
			Point(0,5)
				   };
*/

/*

	Point points1[]={
			Point(0,-1),Point(7,-1),Point(7,4),Point(8,4),Point(8,5),
			Point(6,5),Point(6,0),Point(4,0),Point(4,4),Point(5,4),Point(5,5),
			Point(3,5),Point(3,0),Point(1,0),Point(1,4),Point(2,4),Point(2,5),
			Point(0,5)
				   };

	Polygon pgn(points1, points1+18);
*/


//		Point points[]={Point(0,0),Point(4,0),Point(4,4),Point(0,4)};
//		Polygon pgn(points, points+8);

//		char *filename="Polygon.txt";
//	return ReturnPolygonFromFile("polygon.txt");

	return map;
}


/**
 * Returns the visibility polygon of the robot.
 * @return
 */
Polygon MapIOHandler::GetVisibilityPolygon(){

//	Point points[] = {Point(5,7),Point(6,8),Point(7,8),Point(7,9),Point(5,9)};
//	Point points[] = {Point(8,5),Point(6,5),Point(6,3),Point(7,4),Point(8,4)};

//	Polygon pgn(points, points+5);

//	return ReturnPolygonFromFile("vpolygon.txt");
	return vPolygon;
}

/**
 * Returns the Position of the robot
 * @return
 */
Point MapIOHandler::GetRobotPosition(){

//	Point points[] = {Point(5,7),Point(6,8),Point(7,8),Point(7,9),Point(5,9)};
//	Point points[] = {Point(8,5),Point(6,5),Point(6,3),Point(7,4),Point(8,4)};

//	Polygon pgn(points, points+5);

//	return ReturnPolygonFromFile("vpolygon.txt");
	return robotPos;
}

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
		cout <<"the n is "<<n;
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

