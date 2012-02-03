//============================================================================
// Name        : RobotLocalization.cpp
// Author      : Apurv Verma, Prateek Garg, Kumar Ashwini
// Version     :
// Copyright   : Undergraduate Term Project 2011-2012
// Description : Hello World in C++, Ansi-style
//============================================================================

#include "constants.h"
#include "MapIOHandler.h"
#include "PolygonUtil.h"
#include "HypothesisGenerator.h"
#include "Majoritymap.h"
#include "UIutil.h"
#include <iostream>

#define WIDTH 640
#define HEIGHT 480

using namespace std;


// initialising the polygons
//----------------------------------------------------------------------------------------------

MapIOHandler handle 	= MapIOHandler();
PolygonUtil pUtil		= PolygonUtil();
Polygon mapP			= handle.GetMapPolygon();
Polygon visP            =handle.GetVisibilityPolygon();
//Point robotPos			= Point(0.1,-1.8);
Point robotPos			= Point(7,9);

int no_of_hypothesis=4;
Point points[]={Point(1,1),Point(1,3),Point(3,1),Point(3,3)};
Point center(1,1);

//-----------------------------------------------------------------------------------------------


void display()
{

//	Majoritymap mmap(no_of_hypothesis,points,center,mapP);
//	mmap.GenerateMajorityMap();
//	mmap.PrintMajorityMap();

/*
	HypothesisGenerator g = HypothesisGenerator(mapP, visP, robotPos);
	list<Point> hyps = g.GenHypothesis();
	list<Point>::iterator it;

	for(it = hyps.begin(); it != hyps.end(); it++){
		cout << *it << "\n";
	}
	cout <<"\n" <<hyps.size();
*/

    GLuint listID=tessellate1(mapP,0);
    glCallList(listID);


//   GLuint visID=tessellate1(visP,3);
//   glCallList(visID);
    glFlush();
}


int main(int argc, char ** argv) {

	  graph_t g = pUtil.PrepareVisibilityGraph(mapP);

	  glutInit( &argc, argv );
	  glutInitDisplayMode( GLUT_RGB | GLUT_SINGLE );
	  glutInitWindowSize (WIDTH, HEIGHT);
	  glutInitWindowPosition (50, 100);
	  glutCreateWindow ("Visibility Polygon");

	  init();
	  //Register the GLUT callbacks
	  glutDisplayFunc( display );

	  glutMainLoop();

	return 0;
}


