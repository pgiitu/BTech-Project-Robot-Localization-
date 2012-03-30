/*
 * Grid.cpp
 *
 *  Created on: Mar 25, 2012
 *      Author: ashwani
 */
#include "Grid.h"
#include "math.h"
#define E 0.0001
using namespace std;

Grid::Grid(int k,Point p,float r,list<Segment> listOfSegment)
{
	noOfHyps=k;
	center=p;
	radius=r;
	this->listOfSegment=listOfSegment;
}

void Grid::findIntersection1 (list<Point> &result,float r)
{
	list<Segment>::iterator it;
	float resolution=2*r/(noOfHyps);

//	cout<<"Radius "<<r<<" Resolution "<<resolution<<"\n";
	float bottomX=(center.cartesian(0))-r;
	float bottomY=(center.cartesian(1))-r;
	float topX=(center.cartesian(0))+r;
	float topY=(center.cartesian(1))+r;

//	cout<<"BottomX "<<bottomX <<" BottomY "<<bottomY <<" Topx" <<topX<<" TopY "<<topY<<"\n";
	for(it=listOfSegment.begin();it!=listOfSegment.end();++it)
	{
		Segment seg=*it;
		/**
		 * For Vertical Lines
		 */
//		cout<<"Important Segment "<<seg<<"\n";
		for (int i=1;i<noOfHyps;i++)
			{
				Point p1(bottomX+i*resolution,bottomY);
				Point p2(bottomX+i*resolution,topY);

				Segment s(p1,p2);

//				cout<<"Segment Of Grid "<<s<<"\n";
				Point intersectionPoint;
				if(LineIntersect(seg,s,intersectionPoint))
				{
//					cout<<"Intersection Point "<<intersectionPoint<<"\n";
/*
				float y=intersectionPoint.cartesian(1);
				float height=fabs(y-bottomY);
				float rem=fmod(height,resolution);

				if (rem<=(resolution/2))
					intersectionPoint.cartesian(1)=y-rem;
				else
					intersectionPoint.cartesian(1)=y+resolution-rem;
	*/
					result.push_back(intersectionPoint);
				}
			}

		/**
		 * For Horizontal Lines
		 */
		for (int i=1;i<noOfHyps;i++)
			{
				Point p1(bottomX,bottomY+i*resolution);
				Point p2(topX,bottomY+i*resolution);

				Segment s(p1,p2);

//				cout<<"Segment Of Grid "<<s<<"\n";
				Point intersectionPoint;
				if(LineIntersect(seg,s,intersectionPoint))
				{
//					cout<<"Intersection Point "<<intersectionPoint<<"\n";
				/*
				float x=intersectionPoint.cartesian(0);
				float width=fabs(x-bottomX);
				float rem=fmod(width,resolution);

				if (rem<=(resolution/2))
					intersectionPoint.cartesian(0)=x-rem;
				else
					intersectionPoint.cartesian(0)=x+resolution-rem;
					*/
					result.push_back(intersectionPoint);
				}
			}
	}
}

/*
   Determine the intersection point of two line segments
   Return FALSE if the lines don't intersect
*/
int  Grid::LineIntersect(Segment seg, Segment s1, Point &result)
{
	Point segSource=seg.source();
	Point segDest=seg.target();

	Point s1Source=s1.source();
	Point s1Dest=s1.target();

	double x1=segSource.cartesian(0);
	double y1=segSource.cartesian(1);
	double x2=segDest.cartesian(0);
	double y2=segDest.cartesian(1);

	double x3=s1Source.cartesian(0);
	double y3=s1Source.cartesian(1);
	double x4=s1Dest.cartesian(0);
	double y4=s1Dest.cartesian(1);
	double x,y;



   double mua,mub;
   double denom,numera,numerb;

   denom  = (y4-y3) * (x2-x1) - (x4-x3) * (y2-y1);
   numera = (x4-x3) * (y1-y3) - (y4-y3) * (x1-x3);
   numerb = (x2-x1) * (y1-y3) - (y2-y1) * (x1-x3);

   /* Are the line coincident? */
   if (fabs(numera) < E && fabs(numerb) < E && fabs(denom) < E) {
      x = (x1 + x2) / 2;
      y = (y1 + y2) / 2;
      return 0;
   }

   /* Are the line parallel */
   if (fabs(denom) < E) {
      x = 0;
      y = 0;
      return 0;
   }

   /* Is the intersection along the the segments */
   mua = numera / denom;
   mub = numerb / denom;
   if (mua < 0 || mua > 1 || mub < 0 || mub > 1) {
      x = 0;
      y = 0;
      return 0;
   }
   x = x1 + mua * (x2 - x1);
   y = y1 + mua * (y2 - y1);
   result=Point(x,y);
   return 1;
}




void Grid::findQh(list<Point> &result)
{
	float rmax=radius*noOfHyps;
	int i=0;
	float r=radius;
	while (r<rmax)
	{
		findIntersection1(result,r);
		i++;
		r=pow(2.0,i)*radius;
	}

	//cout <<"Number Of Times "<<i<<"\n";
	list<Segment>::iterator it;
	for (it=listOfSegment.begin();it!=listOfSegment.end();++it)
	{
		Segment s=*it;
		result.push_back(s.source());
		result.push_back(s.target());
	}

}
