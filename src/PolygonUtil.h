/*
 * PolygonUtil.h
 *
 *  Created on: 13-Jan-2012
 *      Author: apurv
 */
#include "constants.h"
#include <boost/numeric/ublas/vector.hpp>
#include <boost/numeric/ublas/io.hpp>


#ifndef POLYGONUTIL_H_
#define POLYGONUTIL_H_

//Point robotPos(0.1,-1.8);  //point from where we are calculating the visibility polygon

class PolygonUtil {
public:
	PolygonUtil();
	virtual ~PolygonUtil();
	Point Right(Polygon& polygon, Point& point);
	Point Left(Polygon& polygon, Point& point);
	bool IsReflex(Polygon& polygon, Point& point);
	void DisplayPolygon(Polygon& polygon);
	Polygon CalcVisibilityPolygon(Polygon& map, Point& point);

	void FindCandidateIntrPoints(Polygon &Map,Ray &rayToCorner,std::list<Point> &intersectionPolygon);

	bool IsVertexOfPolygon(Polygon &Map,Point &p);
//	bool CompareDistance(Point p1,Point p2);

	bool Equals(Point& p1, Point& p2);
	Polygon SortPolygon(Polygon& polygon, Polygon& map);

	bool IsPointOnSegment(Point source, Point target, Point check);

	Polygon VisiblePointSet(Polygon& polygon, Point& point);

	Polygon clonePolygon(Polygon& polygon);

	void insertBefore(Polygon& polygon, VertexIterator& vi, Point& spurPoint);

	template< typename T >
	T scalar_cross_product( const boost::numeric::ublas::vector< T > &a,
			const boost::numeric::ublas::vector< T > &b );


	bool CheckInside(Point& pt,Polygon& polygon);
	//Functions to calculate shortest path and edge visibility.
	bool IsInsidePolygon(Point& p1, Point& p2, Polygon& polygon);
	graph_t PrepareVisibilityGraph(Polygon& map);
};

#endif /* POLYGONUTIL_H_ */