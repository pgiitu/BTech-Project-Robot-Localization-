/*
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


 * this integer is used to decide the phase of the algorithm to be displayed in the glui Window.
 * phase=1  Displaying the Map
 * phase=2 Displaying the Visibility Polygon
 * phase =3 Displaying the Set of Hypothesis

int phase=0;
int listIDMap;
int listIDVPolygon;
int listHypothesis;
int listmmap;
Majoritymap mmap;

// initialising the polygons
//----------------------------------------------------------------------------------------------

MapIOHandler handle 	= MapIOHandler("Scenario1.txt");
PolygonUtil pUtil		= PolygonUtil();
Polygon mapP			= handle.GetMapPolygon();
Polygon visP            =handle.GetVisibilityPolygon();
Point robotPos			= handle.GetRobotPosition();
Segment seg(Point(7,4),Point(8,4));
Polygon VPEdge;
HypothesisGenerator g;
list<Point> hyps;  //list of hypothesis
int no_of_hypothesis;


//For Majority Map and Face Containing Origin
list<Polygon> polygonList;
//-----------------------------------------------------------------------------------------------

void DisplayMajorityMap(Majoritymap mmap);
void DisplayFaceContainingOrigin(Majoritymap mmap);
void DisplayVisibilityPolygonEdge();
void DisplayLocalMap();
void DisplayMajorityMap();

void DisplayFaceContainingOrigin(Majoritymap mmap)
{
	polygonList=mmap.findRegionContaningOrigin();
	glColor3f(0,1,0);
	std::list<Polygon>::iterator pit;
	for(pit=polygonList.begin();pit!=polygonList.end();++pit)
	{
		GLuint j=tessellate1(*pit,0);
		glCallList(j);
	}
}

void DisplayVisibilityPolygonEdge()
{
	if(!VPEdge.size())
	{
		VPEdge=pUtil.CalcWindowEdge(mapP,seg);
		pUtil.DisplayPolygon(VPEdge);
	}
		glColor3f(1,0,0);
		glCallList(listIDMap);
		glColor3f(0,1,0);
		GLuint i=tessellate1(VPEdge,0);
		glCallList(i);
}

void DisplayLocalMap()
{
	glColor3f(0,1,0);
	listIDVPolygon=tessellate1(visP,0);
	glCallList(listIDVPolygon);
}


void DisplayHypothesis()
{
	if(!hyps.size())
	{
		g = HypothesisGenerator(mapP, visP, robotPos);
		hyps = g.GenHypothesis();
	}
	listIDMap=tessellate1(mapP,0);
	glColor3f(1,0,0);
	glCallList(listIDMap);
	glPointSize(10);
	glColor3f(0,0,1);
	glVertex2f(10,2);
	glVertex2f(11,2);
	GLfloat x,y;

    std::list<Point>::iterator it;
	for(it=hyps.begin();it!=hyps.end();++it)
	{
		x=(*it).cartesian(0);
		y=(*it).cartesian(1);
		std::cout<<x<<"   "<<y<<std::endl;
		glVertex2f(x,y);
	}

}

void DisplayMap()
{
	listIDMap=tessellate1(mapP,0);
	listIDVPolygon=tessellate1(visP,0);
	glColor3f(1,0,0);
	glCallList(listIDMap);
	glColor3f(0,1,0);
	glCallList(listIDVPolygon);
}
void display()
{
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	std::list<Point>::iterator it;
	std::list<Faces>::iterator fit;
	switch(phase)
	{
		case 1:
			DisplayMap();
			break;
		case 2:
			DisplayLocalMap();
			break;
		case 3:
			DisplayHypothesis();
			break;
		case 4:
			DisplayMajorityMap(mmap);
			break;
		case 5:
			DisplayFaceContainingOrigin(mmap);
			break;
		case 6:
			DisplayVisibilityPolygonEdge();
			break;

		default:
			break;
	}

	glFlush();

}

void DisplayMajorityMap(Majoritymap mmap)
{
	list<Point>::iterator it;
	int i=0;
	Point hypothesisArray[no_of_hypothesis];
	for(it = hyps.begin(); it != hyps.end(); it++){
		hypothesisArray[i++]=*it;
		cout << "\nHypothesis  "<<*it << "\n";
	}

	Majoritymap mmap1(no_of_hypothesis,hypothesisArray,robotPos,mapP);
	mmap1.GenerateMajorityMap();
	mmap=mmap1;

	std::list<Faces>::iterator fit;
	for(fit=mmap.listMmapFaces.begin();fit!=mmap.listMmapFaces.end();++fit)
	{
		GLuint j=tessellate1((*fit).face,0);
		if((*fit).partOfMajorityMap)
		{
			glColor3f(0,1,0);
		}
		else
		{
			glColor3f(1,0,0);
		}
		glCallList(j);
	}
}


void ShowMap(int ID)
{
	switch(ID)
	{
		case 1:
			phase=ID;  //ID=1
			glutPostRedisplay();
			break;
		case 2:
			phase=ID;   //ID=2
			glutPostRedisplay();
			break;
		case 3:
			phase=ID;  //ID=3
			glutPostRedisplay();
			break;
		case 4:
			phase=ID;	//ID=4
			glutPostRedisplay();
			break;
		case 5:
			phase=ID;	//ID=5
			glutPostRedisplay();
			break;
		case 6:
			phase=ID;	//ID=6
			glutPostRedisplay();
			break;
		default :
			break;
	}
}


int main(int argc, char ** argv) {


	Polygon p;
	cout<<"Size is :- "<<p.size()<<"\n";

	HypothesisGenerator g1 = HypothesisGenerator(mapP, visP, robotPos);
	list<Point> hyps1 = g1.GenHypothesis();

    no_of_hypothesis=hyps.size();

    std::cout<<"Number Of Hypothesis"<<hyps.size();


	list<Point>::iterator it;
	int i=0;
	Point hypothesisArray[no_of_hypothesis];
	for(it = hyps.begin(); it != hyps.end(); it++){
		hypothesisArray[i++]=*it;
		cout << "\nHypothesis  "<<*it << "\n";
	}

	Majoritymap mmap1(no_of_hypothesis,hypothesisArray,robotPos,mapP);
	mmap1.GenerateMajorityMap();
	mmap=mmap1;
	mmap.findRegionContaningOrigin();

//	pUtil.DisplayPolygon(mapP);

	VPEdge=pUtil.CalcWindowEdge(mapP,seg);
    pUtil.DisplayPolygon(VPEdge);



 * Calculating the Fij's and Gij's


    list<Polygon> KPolygonList;  //For storing the Kij's

    for(i=0;i<no_of_hypothesis;i++)
    {
    	Point center=hypothesisArray[i];
    	list<Polygon> GPolygonList;  //for storing the gij's

    	for(int j=0;j<no_of_hypothesis;j++)
    	{
    		Majoritymap overlayArrangement;
    		list<Polygon> TwoPolygons;
    		Vector t(robotPos.cartesian(0)-center.cartesian(0),robotPos.cartesian(1)-center.cartesian(1));
    		Transformation trans(CGAL::TRANSLATION,t);
    		TwoPolygons.push_back(overlayArrangement.GetTranslatePolygon(trans,mapP));
    		if(i!=j )
    		{
    			Vector t1(robotPos.cartesian(0)-hypothesisArray[j].cartesian(0),robotPos.cartesian(1)-hypothesisArray[j].cartesian(1));
        		Transformation trans1(CGAL::TRANSLATION,t1);
        		TwoPolygons.push_back(overlayArrangement.GetTranslatePolygon(trans1,mapP));
        		overlayArrangement.GenerateOverlay(TwoPolygons);
        		Polygon Fij=overlayArrangement.OverlayContaningOrigin(robotPos);

        		cout<<"For Hypothesis "<<i<< "  " <<hypothesisArray[i]<<"  hypothesis  "<<j<<"  "<<hypothesisArray[j]<<endl;
        		pUtil.DisplayPolygon(Fij);

        		Polygon Gij=pUtil.CalcGPolygon(robotPos,Fij,TwoPolygons);
        		GPolygonList.push_back(Gij);
    		}
    	}



    	 * Using GPOLYGONLIST we need to construct Ki
    	 * Insert Ki in KPOLYGONLIST


    	cout<<"Hypothesis  "<<hypothesisArray[i]<<endl;
    	list<Polygon>::iterator pit;
    	for(pit=GPolygonList.begin();pit!=GPolygonList.end();++pit)
    	{
    		pUtil.DisplayPolygon(*pit);
    	}

    	Majoritymap Ki(GPolygonList.size(),GPolygonList);
    	Ki.GenerateOverlay(GPolygonList);
    	Ki.partMajority();


    	//Ki.PrintMajorityMap();

    }




	 *
	 *  VISIBILITY POLYGON OF A POINT IT REQUIRES POINT UPGRADATION
	 *  AT TWO POINTS.
	 *
	 *  THERE IS AN ERROR IN HYPOTHESIS GENERATION IN ISMATCH FUNCTION
	 *
	 *



	    HypothesisGenerator g = HypothesisGenerator(mapP, visP, robotPos);
		list<Point> hyps = g.GenHypothesis();
		list<Point>::iterator it;

	    int no_of_hypothesis=hyps.size();

	    std::cout<<"Number Of Hypothesis"<<hyps.size();
	    Point hypothesisArray[no_of_hypothesis];

	    int i=0;
		for(it = hyps.begin(); it != hyps.end(); it++){
			hypothesisArray[i++]=*it;
			cout << "\nHypothesis  "<<*it << "\n";
		}

		Majoritymap mmap(no_of_hypothesis,hypothesisArray,robotPos,mapP);
		mmap.GenerateMajorityMap();
		polygonList=mmap.findRegionContaningOrigin();


		list<Polygon> KPolygonList;

		for(int i=0;i<no_of_hypothesis;i++)
		{
			Point center=hypothesisArray[i];

			list<Polygon> GPolygonList;
			list<Polygon> PPolygonList;

			Majoritymap overlayArrangement;



			Vector T((robotPos.cartesian(0)-center.cartesian(0)),(robotPos.cartesian(1)-center.cartesian(1)));
			Transformation translate(CGAL::TRANSLATION, T);
			PPolygonList.push_back(overlayArrangement.GetTranslatePolygon(translate,mapP));

			for(int j=0;j<no_of_hypothesis;j++)
			{
				if(i!=j)
				{
					Vector T1((robotPos.cartesian(0)-hypothesisArray[j].cartesian(0)),(robotPos.cartesian(1)-hypothesisArray[j].cartesian(1)));
					Transformation translate1(CGAL::TRANSLATION, T1);
				    PPolygonList.push_back(overlayArrangement.GetTranslatePolygon(translate1,mapP));

				    overlayArrangement.GenerateOverlay(PPolygonList);
				    Polygon FPolygon=overlayArrangement.OverlayContaningOrigin(center);

				    Polygon GPolygon=pUtil.CalcGPolygon(center,FPolygon,PPolygonList);
					GPolygonList.push_back(GPolygon);
				}
			}

			overlayArrangement.GenerateOverlay(GPolygonList);
			overlayArrangement.partMajority();


			 * Run an iterator over all the faces of
			 * overlay object and take the union of all these polygons
			 * to form Ki and push it into the list of KPolygons

		 	 list<Faces>::iterator f;

			  std::cout<<"Number Of faces"<<overlayArrangement.listMmapFaces.size()<<"\n";

			  Polygon_with_holes unionR=Polygon_with_holes((overlayArrangement.listMmapFaces.begin())->face);

			  for(f=overlayArrangement.listMmapFaces.begin();f!=overlayArrangement.listMmapFaces.end();++f)
			  {
				  if(CGAL::join(unionR,f->face,unionR))
				  {
					  std::cout<<"The Union \n";
				  }

			  }

			  Polygon KPolygon;
			  for(VertexIterator vi=unionR.outer_boundary().vertices_begin();vi!=unionR.outer_boundary().vertices_end();++vi)
			  {
				  KPolygon.push_back(*vi);
			  }

			  KPolygonList.push_back(KPolygon);

		}



   std::set<Point> s;
   s.insert(Point(2,1));
   s.insert(Point(2,1));
   s.insert(Point(2,1));
   s.insert(Point(4,4));
   s.insert(Point(21,11));
   s.insert(Point(22,11));

   std::set<Point>::iterator itt;
   for(itt=s.begin();itt!=s.end();++itt)
   {
	   cout<<*itt<<endl;
   }





    GLUI_Panel *HGererationPanel;

	  glutInit( &argc, argv );
	  glutInitDisplayMode( GLUT_RGB | GLUT_SINGLE );
	  glutInitWindowSize (WIDTH, HEIGHT);
	  glutInitWindowPosition (50, 100);
	  int windowId=glutCreateWindow ("Visibility Polygon");

	  init();
	  glutDisplayFunc( display );

	  GLUI_Master.set_glutIdleFunc (NULL);
	  * Now create a GLUI user interface window and add controls *

	  GLUI *glui = GLUI_Master.create_glui( "GLUI", 0 );
	  glui->set_main_gfx_window( windowId );
	  glui->add_button("Show Map",1,ShowMap);
	  glui->add_button("Show Visibility Map",2,ShowMap);
	  HGererationPanel=glui->add_panel("Hypohesis Generation");
	  glui->add_button_to_panel(HGererationPanel,"Hypothesis Set",3,ShowMap);
	  glui->add_button("Show Majority Map",4,ShowMap);
	  glui->add_button("Face Containing Origin",5,ShowMap);
	  glui->add_separator();
	  glui->add_button("Visibility Polygon of edge",6,ShowMap);
	  glutMainLoop();

      return 0;
}



*/
