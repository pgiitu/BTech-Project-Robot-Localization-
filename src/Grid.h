/**
 * Grid.h
 *
 *  Created on: Mar 25, 2012
 *      Author: Ashwani, Apurv, Prateek
 */

#ifndef GRID_H_
#define GRID_H_
#include "constants.h"
class Grid{
	public:
		int noOfHyps;
		Point center;
		float radius;
		std::list<Segment> listOfSegment;

		Grid(int k,Point p,float r,std::list<Segment> l1);
		void findIntersection1(std::list<Point> &result,float r);
		int LineIntersect(Segment seg, Segment s1, Point &result);

		void findQh(std::list<Point> &result);
};



#endif /* GRID_H_ */
