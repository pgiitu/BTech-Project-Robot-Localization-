/**
 * PolygonUtil.h
 *
 *  Created on: 13-Jan-2012
 *      Author: apurv,ashwani,prateek
 */
#include "constants.h"
#include <boost/numeric/ublas/vector.hpp>
#include <boost/numeric/ublas/io.hpp>


#ifndef POLYGONUTIL_H_
#define POLYGONUTIL_H_


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

	bool Equals(Point& p1, Point& p2);
	Polygon SortPolygon(Polygon& polygon, Polygon& map);

	bool IsPointOnSegment(Point source, Point target, Point check);

	Polygon VisiblePointSet(Polygon& polygon, Point& point);
	Polygon CalcVisibilityPolygonEdge(Polygon& map, Segment& seg,graph_t& g, Point vertex[]);

	Polygon clonePolygon(Polygon& polygon);

	void insertBefore(Polygon& polygon, VertexIterator& vi, Point& spurPoint);

	template< typename T >
	T scalar_cross_product( const boost::numeric::ublas::vector< T > &a,
			const boost::numeric::ublas::vector< T > &b );

	template< typename T >
	T dot_product( const boost::numeric::ublas::vector< T > &a,
			const boost::numeric::ublas::vector< T > &b );

	Polygon CalcWindowEdge(Polygon& map, Segment s);

	bool CheckInside(Point& pt,Polygon& polygon);
	int findIndex(Point vertex[],int size,Point point);
	Polygon CalcGPolygon(Point& center,Polygon& FPolygon,std::list<Polygon> polygonList);
	int ClassifyEdges(Segment edge, Polygon& P1,Polygon& P2);

	bool EqualsValue(double v1, double v2);
	bool doPolygonsMatch(Polygon &P1,Polygon &P2);
	double GetSlope(Segment& edge);
	Polygon RemoveCollinearPoints(Polygon P);


	bool IsInsidePolygon(Point& p1, Point& p2, Polygon& polygon);
	bool IsInsidePolygon1(Point& p1, Point& p2, Polygon& polygon);
	graph_t PrepareVisibilityGraph(Polygon& map, Point vertex[]);

	bool isPathIntersect(Point vertex[],std::list<int> path1,std::list<int> path2);

	std::list<Polygon> CalcAllVisibilityPolygon(Polygon& map,Polygon& P1,Polygon& P2);

	std::vector<vertex_descriptor> CalcShortestPath(int source,graph_t& g,Point vertex[]);
	int path(std::vector<vertex_descriptor> pathVector,int source,int target,std::list<int>& pathList);
	bool isPathConvex(Point vertex[],std::list<int> pathList,boost::numeric::ublas::vector<double> edge);
	int findCommonAncestor(std::list<int> path1,std::list<int> path2);
	std::list<Point> UniqueList(std::list<Point> pointList);

	bool isPointInList(std::list<Point> pointList,Point p);

	std::list<Polygon> unionPolygons(std::list<Polygon>);
	Polygon convertPolygonWithHolesToPolygon(Polygon_with_holes polyHoles);

	double minimumDistance(Segment edge,Point p);


};

#endif /* POLYGONUTIL_H_ */
