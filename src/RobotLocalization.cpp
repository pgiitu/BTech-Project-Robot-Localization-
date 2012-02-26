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
#include <string>

#define WIDTH 640
#define HEIGHT 480

using namespace std;

 string sh1,sh2;
 GLUI_String shyp1,shyp2;
 GLUI_Panel *FijPanel;



 /*
  * for Fij's
  */
 list<Majoritymap> Kijs;
 list<list<Polygon> > Gijs;
 list<list<Polygon> > Fijs;
 list<list<list<Polygon> > > all_two_faces;




/*
 * this integer is used to decide the phase of the algorithm to be displayed in the glui Window.
 * phase=1  Displaying the Map
 * phase=2 Displaying the Visibility Polygon
 * phase =3 Displaying the Set of Hypothesis
 */

int phase=0;
int listIDMap;
int listIDVPolygon;
int listHypothesis;
int listmmap;
Majoritymap mmap;

// initialising the polygons
//----------------------------------------------------------------------------------------------

MapIOHandler handle 	= MapIOHandler("Scenario4.txt");
PolygonUtil pUtil		= PolygonUtil();
Polygon mapP			= handle.GetMapPolygon();
Polygon visP            =handle.GetVisibilityPolygon();
Point robotPos			= handle.GetRobotPosition();
Segment seg(Point(7.5,-5),Point(8,-5));
Point point(5.5,-5);
Polygon VPEdge;
HypothesisGenerator g;
list<Point> hyps;  //list of hypothesis
int no_of_hypothesis;


//For Majority Map and Face Containing Origin
list<Polygon> polygonList;
//-----------------------------------------------------------------------------------------------

void DisplayMajorityMap();
void DisplayFaceContainingOrigin();
void DisplayVisibilityPolygonEdge();
void DisplayLocalMap();
void DisplayMajorityMap();



void DisplayMap()
{
	listIDMap=tessellate1(mapP,0);
	listIDVPolygon=tessellate1(visP,0);
	glColor3f(1,0,0);
	glCallList(listIDMap);
	glColor3f(0,1,0);
	glCallList(listIDVPolygon);
}

void DisplayLocalMap()
{
	glColor3f(0,1,0);
	listIDVPolygon=tessellate1(visP,0);
	glCallList(listIDVPolygon);
}

void CalcHypothesis()
{
//	if(!hyps.size())
//	{
	list<Point>::iterator it;
		g = HypothesisGenerator(mapP, visP, robotPos);
		hyps = g.GenHypothesis();
		cout<<"list length "<<hyps.size()<<endl;
		for(it = hyps.begin(); it != hyps.end(); it++)
		{
			cout << "\nHypothesis  "<<*it << "\n";
		}

//	}
}

void DisplayHypothesis()
{
	CalcHypothesis();
	listIDMap=tessellate1(mapP,0);
	glColor3f(1,0,0);
	glCallList(listIDMap);
	glPointSize(10);
	glColor3f(0,0,1);
	glBegin(GL_POINTS);
    std::list<Point>::iterator it;
	for(it=hyps.begin();it!=hyps.end();++it)
	{
		glVertex2f((*it).cartesian(0),(*it).cartesian(1));
	}
	glEnd();
}

void CalcMajorityMap()
{
		CalcHypothesis();
		list<Point>::iterator it;
		int i=0;
		no_of_hypothesis=hyps.size();
		Point hypothesisArray[no_of_hypothesis];
		for(it = hyps.begin(); it != hyps.end(); it++)
		{
			hypothesisArray[i++]=*it;
//			cout << "\nHypothesis  "<<*it << "\n";
		}

		Majoritymap mmap1(no_of_hypothesis,hypothesisArray,robotPos,mapP);
		mmap1.GenerateMajorityMap();
		mmap=mmap1;
}

void DisplayMajorityMap()
{
	CalcMajorityMap();
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
			glColor3f(0,0,0);
		}
		glCallList(j);
	}
}
void CalcFaceContainingOrigin()
{
	CalcMajorityMap();
	polygonList=mmap.findRegionContaningOrigin();
}
void DisplayFaceContainingOrigin()
{
	CalcFaceContainingOrigin();
	glColor3f(0,1,0);
	std::list<Polygon>::iterator pit;
	for(pit=polygonList.begin();pit!=polygonList.end();++pit)
	{
		GLuint j=tessellate1(*pit,0);
		glCallList(j);
	}
}

void CalcVisibilityPolygonEdge()
{
	if(!VPEdge.size())
	{
		VPEdge=pUtil.CalcWindowEdge(mapP,seg);
//		pUtil.DisplayPolygon(VPEdge);
	}
}
void DisplayVisibilityPolygonEdge()
{
		CalcVisibilityPolygonEdge();
		glColor3f(0,0,0);
		listIDMap=tessellate1(mapP,0);
		glCallList(listIDMap);
		glColor3f(0,1,0);
		GLuint i=tessellate1(VPEdge,0);
		glCallList(i);

		glPointSize(10);
		glColor3f(0,0,1);
		glBegin(GL_LINES);
		Point s=seg.source();
		Point e=seg.end();
//		cout<<s<<"\n\n\n\n\n\n";
		glVertex2f(s.cartesian(0),s.cartesian(1));
		glVertex2f(e.cartesian(0),e.cartesian(1));
		glEnd();
}

void DisplayVisibilityPolygonPoint()
{
	Polygon p1=pUtil.CalcVisibilityPolygon(mapP,point);
	cout<<"Visibility Polygon of Point \n";
	pUtil.DisplayPolygon(p1);
	p1=pUtil.RemoveCollinearPoints(p1);
	cout<<" removed Visibility Polygon of Point \n";
	pUtil.DisplayPolygon(p1);
	glColor3f(0,0,0);
	listIDMap=tessellate1(mapP,0);
	glCallList(listIDMap);
	glColor3f(0,1,0);
	GLuint i=tessellate1(p1,0);
	glCallList(i);
	glBegin(GL_POINTS);
	glPointSize(4);
	glColor3f(0,0,1);
	glVertex2f(point.cartesian(0),point.cartesian(1));
//	glVertex2f(10,10);
	glEnd();
//	glFlush();
}


void CalculateFijs()
{
	CalcFaceContainingOrigin();
	Point hypothesisArray[no_of_hypothesis];
	list<Point>::iterator it;
    int i=0;

    for(it = hyps.begin(); it != hyps.end(); it++)
	{
		hypothesisArray[i++]=*it;
	}

    //list<list<Polygon>> AllKij;


    list<Polygon> KPolygonList;  //For storing the Kij's
    for(i=0;i<no_of_hypothesis;i++)
    {
    	Point center=hypothesisArray[i];
    	list<Polygon> GPolygonList;  //for storing the gij's
    	list<Polygon> FPolygonList;  //for storing the gij's
//    	list<list<Polygon>> two_faces;
    	list<list<Polygon> > two_faces;
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

//        		cout<<"For Hypothesis "<<i<< "  " <<hypothesisArray[i]<<"  hypothesis  "<<j<<"  "<<hypothesisArray[j]<<endl;
//        		pUtil.DisplayPolygon(Fij);

        		Polygon Gij=pUtil.CalcGPolygon(robotPos,Fij,TwoPolygons);

//        		cout<<"Pushed\n\n";
        		two_faces.push_back(TwoPolygons);
        		FPolygonList.push_back(Fij);
        		GPolygonList.push_back(Gij);
    		}
    	}

    	Gijs.push_back(GPolygonList);
    	Fijs.push_back(FPolygonList);
    	all_two_faces.push_back(two_faces);
    	 // Using GPOLYGONLIST we need to construct Ki
    	 // Insert Ki in KPOLYGONLIST


//    	cout<<"Hypothesis  "<<hypothesisArray[i]<<endl;
    	list<Polygon>::iterator pit;
    	for(pit=GPolygonList.begin();pit!=GPolygonList.end();++pit)
    	{
//    		pUtil.DisplayPolygon(*pit);
    	}

    	Majoritymap Ki(GPolygonList.size(),GPolygonList);
    	Ki.GenerateOverlay(GPolygonList);
    	Ki.partMajority();

    	Kijs.push_back(Ki);
    	//Ki.PrintMajorityMap();
    }

}

void Display_Fij()
{
	list<list<list<Polygon> > >::iterator pit3=all_two_faces.begin();
	cout<<"Length of all two faces is :- "<<all_two_faces.size()<<"\n";
	cout<<"Length of all Fij is :- "<<Fijs.size()<<"\n";
	cout<<"Length of all Gij is :- "<<Gijs.size()<<"\n\n";
	list<list<Polygon> >::iterator pit2=(*pit3).begin();
	list<Polygon> ::iterator pit1;
	glColor3f(1,0,0);
//	cout<<"Finally5555\n";
	for(pit1=(*pit2).begin();pit1!=(*pit2).end();++pit1)
	{
//		cout<<"Finally\n";
//		pUtil.DisplayPolygon(*pit1);
		GLuint i=tessellate1((*pit1),0);
		glCallList(i);
	}
	(*pit2).pop_front();


	glColor3f(0,1,0);
	list<list<Polygon> >::iterator fit2=Fijs.begin();
	list<Polygon>::iterator fit1=(*fit2).begin();
	GLuint i=tessellate1((*fit1),0);
	glCallList(i);
	(*fit2).pop_front();

	glColor3f(0,0,1);
	list<list<Polygon> >::iterator git2=Gijs.begin();
	list<Polygon>::iterator git1=(*git2).begin();
	i=tessellate1((*git1),0);
	glCallList(i);
	pUtil.DisplayPolygon(*git1);
	(*git2).pop_front();

	cout<<"GIJ Drawn\n";
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
			DisplayMajorityMap();
			break;
		case 5:
			DisplayFaceContainingOrigin();
			break;
		case 6:
			DisplayVisibilityPolygonEdge();
			break;
		case 7:
			DisplayVisibilityPolygonPoint();
			break;
		case 8:
			CalculateFijs();
			break;
		case 10:
//			cout<<"Fisiisisissssssssssssssssss\n";
			Display_Fij();
			break;
		default:
			break;
	}

	glFlush();

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
		case 7:
			phase=ID;	//ID=6
			glutPostRedisplay();
			break;
		case 8:
			FijPanel->hidden=false;
			phase=ID;
			glutPostRedisplay();
			break;
		case 10:
			phase=ID;
			glutPostRedisplay();
			break;
		default :
			break;
	}
}

string convert_point_string(Point p)
{

	GLUI_String s="(";
//	s.append(p.cartesian(0));
	s.append(",");
//	s.append(p.cartesian(1));
	s.append(")");
	return s;
}


int main(int argc, char ** argv)
{

/*
	Polygon p;
	cout<<"\nSize is :- "<<p.size()<<"\n";

	HypothesisGenerator g = HypothesisGenerator(mapP, visP, robotPos);
	list<Point> hyps = g.GenHypothesis();

    no_of_hypothesis=hyps.size();

    std::cout<<"Number Of Hypothesis"<<hyps.size();


	list<Point>::iterator it;
	int i=0;
	Point hypothesisArray[no_of_hypothesis];
	for(it = hyps.begin(); it != hyps.end(); it++)
	{
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
*/


 // Calculating the Fij's and Gij's
/*
	CalcFaceContainingOrigin();
	Point hypothesisArray[no_of_hypothesis];
	list<Point>::iterator it;
    int i=0;

    for(it = hyps.begin(); it != hyps.end(); it++)
	{
		hypothesisArray[i++]=*it;
	}

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

    	 // Using GPOLYGONLIST we need to construct Ki
    	 // Insert Ki in KPOLYGONLIST


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
*/



	/*
	 *
	 *  VISIBILITY POLYGON OF A POINT IT REQUIRES POINT UPGRADATION
	 *  AT TWO POINTS.
	 *
	 *  THERE IS AN ERROR IN HYPOTHESIS GENERATION IN ISMATCH FUNCTION
	 *
	 *
	 */


/*
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
*/

      GLUI_Panel *HGererationPanel;
	  GLUI_EditText *hyp1,*hyp2;

      glutInit( &argc, argv );
	  glutInitDisplayMode( GLUT_RGB | GLUT_SINGLE );
	  glutInitWindowSize (WIDTH, HEIGHT);
	  glutInitWindowPosition (50, 100);
	  int windowId=glutCreateWindow ("Visibility Polygon");

	  init();
	  glutDisplayFunc( display );

	  GLUI_Master.set_glutIdleFunc (NULL);
	  /** Now create a GLUI user interface window and add controls **/

	  GLUI *glui = GLUI_Master.create_glui( "GLUI", 0 );
	  glui->set_main_gfx_window( windowId );
	  glui->add_button("Show Map",1,ShowMap);
	  glui->add_button("Show Visibility Map",2,ShowMap);
	  HGererationPanel=glui->add_panel("Hypothesis Generation");
	  glui->add_button_to_panel(HGererationPanel,"Hypothesis Set",3,ShowMap);
//	  HGererationPanel->hidden=true;
	  glui->add_button("Show Majority Map",4,ShowMap);
	  glui->add_button("Face Containing Origin",5,ShowMap);
	  glui->add_separator();
	  glui->add_button("Visibility Polygon of edge",6,ShowMap);
	  glui->add_separator();
	  glui->add_button("Visibility Polygon of point",7,ShowMap);

	  glui->add_button("Calculate Fij's",8,ShowMap);
	  FijPanel=glui->add_panel("Calculating Fij's and Gij's");
	  hyp1=glui->add_edittext_to_panel(FijPanel,"Hypothesis 1",shyp1,GLUI_EDITTEXT_STRING);
	  hyp2=glui->add_edittext_to_panel(FijPanel,"Hypothesis 2",shyp2,GLUI_EDITTEXT_STRING);
	  glui->add_button_to_panel(FijPanel,"Start Computing",10,ShowMap);
	  FijPanel->hidden=true;

	  //	  hyp1->text=(string)Point(1,2);
//	  glui->add_statictext("COlor Represents this");
	  glutMainLoop();

      return 0;
}
