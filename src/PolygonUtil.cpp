/*
 * PolygonUtil.cpp
 *
 *  Created on: 13-Jan-2012
 *      Author: apurv
 */

#include "PolygonUtil.h"

#include <iostream>
#include <list>
#include <math.h>
#include <stack>

#include <boost/config.hpp>
#include <boost/tuple/tuple.hpp>
#include <boost/tuple/tuple_io.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/adjacency_list.hpp>

#define E 0.00001
using namespace boost;
//using namespace boost::numeric::ublas;
//using namespace std;
Point fixedPoint;

bool CompareDistance(Point p1,Point p2);
bool CompareAngle(Point p1,Point p2);


PolygonUtil::PolygonUtil() {
	// TODO Auto-generated constructor stub

}

PolygonUtil::~PolygonUtil() {
	// TODO Auto-generated destructor stub
}

/**
 * 0 means a False
 * 1 means a True
 *
 * @param polygon
 * @param point
 * @return
 */
bool PolygonUtil::IsReflex(Polygon &polygon,Point &point){

	Point right = Right(polygon, point);
	Point left  = Left(polygon, point);


	boost::numeric::ublas::vector<double> R(2);
	boost::numeric::ublas::vector<double> L(2);
	boost::numeric::ublas::vector<double> P(2);

	R(0) = right.cartesian(0);
	R(1) = right.cartesian(1);

	L(0) = left.cartesian(0);
	L(1) = left.cartesian(1);

	P(0) = point.cartesian(0);
	P(1) = point.cartesian(1);

	R = R - P;
	L = L - P;

	double prod = scalar_cross_product(R, L);

	return ( prod > 0)?false:true;
}


/**
 *
 * @param polygon
 * @param point
 * @return
 */
Point PolygonUtil::Right(Polygon& polygon, Point& point){

	Point right;
	Point start = *(polygon.vertices_begin());

	for(VertexIterator vi = polygon.vertices_begin(); vi != polygon.vertices_end(); ++vi){

		if( Equals(*vi,point) ){
			right = ( Equals( *vi, *(polygon.vertices_end()-1) ) )?start:*(vi+1);
			break;
		}
	}
	return right;
}


/**
 *
 * @param polygon
 * @param point
 * @return
 */
Point PolygonUtil::Left(Polygon& polygon, Point& point)
{

	Point left;
	Point end = *(polygon.vertices_end()-1);

	for(VertexIterator vi = polygon.vertices_begin(); vi != polygon.vertices_end(); ++vi)
	{

		if( Equals (*vi,point))
		{
			left = ( Equals(*vi,*polygon.vertices_begin() ))?end:*(vi-1);
			break;
		}
	}
	return left;
}


/**
 *
 * @param map
 * @param rayToCorner Ray from the robot's position to the reflex vertex.
 * @param intersectionPolygon
 */
void PolygonUtil::FindCandidateIntrPoints(Polygon &map,Ray &rayToCorner,std::list<Point> &intersectionPolygon)
{
	Point vertexPolygon=rayToCorner.point(1);

	Point intPoint;
	for (EdgeIterator ei = map.edges_begin(); ei != map.edges_end(); ++ei)
	{
		Object obj=CGAL::intersection(rayToCorner,*ei);

		if(CGAL::assign(intPoint,obj))
		{
			if(intPoint != vertexPolygon && !IsVertexOfPolygon(map, intPoint))
			{
				intersectionPolygon.push_back(intPoint);
			}
		}
	}
}


/**
 * Deep copies a polygon.
 * @param polygon
 * @return
 */
Polygon PolygonUtil::clonePolygon(Polygon& polygon){
	Polygon copyPolygon = Polygon();
	for (VertexIterator vi = polygon.vertices_begin(); vi != polygon.vertices_end(); ++vi){
		Point origPoint = *vi;
		Point copyPoint = Point(origPoint.cartesian(0), origPoint.cartesian(1));
		copyPolygon.push_back(copyPoint);
	}
	return copyPolygon;
}




Polygon PolygonUtil::CalcVisibilityPolygon(Polygon& map, Point& point)
{
	Polygon setVisiblePoints = VisiblePointSet(map,point);
	Polygon setVisiblePointsCopy = clonePolygon(setVisiblePoints);

	VertexIterator viCopy = setVisiblePointsCopy.vertices_begin();

	for (VertexIterator vi = setVisiblePoints.vertices_begin(); vi != setVisiblePoints.vertices_end(); ++vi)
	{
		if(IsReflex(map,*vi))
		{
			Ray rayToCorner(point,*vi);
			std::list<Point> intPointList;
			std::list<Point>::iterator it;

			Point p1=*vi;
			Point p2;
			FindCandidateIntrPoints(map,rayToCorner,intPointList);

			if(intPointList.size() > 0){
				//Sort the points in intersectionPolygon according to distance
				intPointList.sort(CompareDistance);
				intPointList.unique();
				Point spuriousVertex = *intPointList.begin();

				p2=spuriousVertex;

				insertBefore(setVisiblePointsCopy, viCopy, spuriousVertex);
			}
		}

		viCopy++;
	}

	setVisiblePointsCopy=SortPolygon(setVisiblePointsCopy,map);
	return setVisiblePointsCopy;
}
/**
 *
 */
bool CompareDistance1(Point p1,Point p2)
{
	if(CGAL::squared_distance(fixedPoint,p1)<CGAL::squared_distance(fixedPoint,p2))
		return true;
	else
		return false;
}
/**
 *
 */
bool PolygonUtil::IsPointOnSegment(Point source, Point target, Point check)
{
	Segment edge(source,target);
	if(check!=target && (CGAL::squared_distance(source,check)<CGAL::squared_distance(source,target)))
	{
		boost::numeric::ublas::vector<double> R(2);
		boost::numeric::ublas::vector<double> L(2);
		boost::numeric::ublas::vector<double> P(2);

		R(0) = source.cartesian(0);
		R(1) = source.cartesian(1);

		L(0) = target.cartesian(0);
		L(1) = target.cartesian(1);

		P(0) =check.cartesian(0);
		P(1) =check.cartesian(1);

		R = R - P;
		L = L - P;

		double prod = scalar_cross_product(R, L);

		if(fabs(prod)<E)
		{
			return true;
		}
	}
	return false;
}
/**
 *
 */
Polygon PolygonUtil::SortPolygon(Polygon& polygon,Polygon& mapP)
{
	Polygon visibilyPolygon;

	for(EdgeIterator ei = mapP.edges_begin(); ei != mapP.edges_end(); ++ei)
	{
		Point source=(*ei).source();
		Point target=(*ei).target();

		std::list<Point> PointList;
		std::list<Point>::iterator it;

		for (VertexIterator vi = polygon.vertices_begin(); vi != polygon.vertices_end(); ++vi)
		{
			Point check=*vi;

			if(IsPointOnSegment(source,target,check))
			{
				PointList.push_back(check);
			}
		}

		fixedPoint=source;
		PointList.sort(CompareDistance1);

		for(it=PointList.begin();it!=PointList.end();++it)
		{
			visibilyPolygon.push_back(*it);
		}
	}
	return visibilyPolygon;
}




void PolygonUtil::insertBefore(Polygon& polygon, VertexIterator& vi, Point& spurPoint){

	polygon.insert(vi, spurPoint);
	vi--;
}


/*
void PolygonUtil::insertAfter(Polygon& polygon, VertexIterator& vi, Point& spurPoint){
	vi++;
	polygon.insert(vi, spurPoint);
	vi--;

}
 */


Polygon PolygonUtil::VisiblePointSet(Polygon& polygon, Point& point)
{
	Polygon result = Polygon();

	//Line segment joining point and corner of polygon
	for (VertexIterator vi = polygon.vertices_begin(); vi != polygon.vertices_end(); ++vi)
	{
		//		std::cout << "\n\n\n\n";
		//		std::cout << "Drawing edge "<< point <<", "<< *vi << "\n";////////////////////////////

		Segment halfedge(point,*vi);
		Point intPoint;

		int flag=0;  //to check if corner point or not
		for (EdgeIterator ei = polygon.edges_begin(); ei != polygon.edges_end(); ++ei)
		{
			//			std::cout << "    On Edge "<< *ei << "\n";///////////////////////////////////////
			Object obj=CGAL::intersection(halfedge,*ei);
			if(CGAL::assign(intPoint,obj))
			{
				//				std::cout << "Inter Point is "<< intPoint<<"\n";/////////////////////
				//				std::cout << "vi is "<< *vi<<"\n";/////////////////////
				//				std::cout << "IsVertexOfPolygon "<< IsVertexOfPolygon(polygon, intPoint)<<"\n";
				//				std::cout << "      Assigned"<<"\n";///////////////////////////
				if( !IsVertexOfPolygon(polygon, intPoint))
				{
					//					std::cout << "     Vertex invisible"<<"\n";
					flag=1;
					break;
				}
			}
			//			else{
			//				std::cout << "Not Assigned \n";
			//			}
		}
		if(flag==0)
		{
			//			std::cout<<"Pushing" << *vi << "as a visible vertex \n";/////////////////////
			result.push_back(*vi);
		}
	}
	return result;
}



bool PolygonUtil::IsVertexOfPolygon(Polygon &map,Point &p)
{
	for (VertexIterator vi=map.vertices_begin(); vi!=map.vertices_end();++vi)
	{
		if( Equals(p,*vi) )
			return true;
	}
	return false;
}



/**
 * Prints the polygon vertices and edges.
 * The iterator always traverses the polygon vertices in anti-clockwise order.//TODO:Check this.
 * @param polygon
 */
void PolygonUtil::DisplayPolygon(Polygon& polygon)
{
	int n = 0;

	std::cout<<"\n\nThe vertices of the polygon are ...\n";

	for(VertexIterator vi = polygon.vertices_begin(); vi != polygon.vertices_end(); ++vi)
	{
		std::cout << "vertex " << n++ << " = " << *vi << "\n";
	}

	std::cout<<"\n\nThe edges of the polygon are ...\n";
	n=0;
	for (EdgeIterator ei = polygon.edges_begin(); ei != polygon.edges_end(); ++ei)
		std::cout << "edge " << n++ << " = " << *ei << "\n";

	std::cout<<"\n\n";
}




bool CompareDistance(Point p1,Point p2)
{
	//	Point startingPoint(0.5,-1.8);
	Point robotPos(0.1,-1.8);  //point from where we are calculating the visibility polygon

	if(CGAL::squared_distance(robotPos,p1)<CGAL::squared_distance(robotPos,p2))
		return true;
	else
		return false;
}


bool PolygonUtil::Equals(Point& p1, Point& p2){
	return ( fabs(p1.cartesian(0) - p2.cartesian(0))<E and fabs(p1.cartesian(1) - p2.cartesian(1))<E );
}



template< typename T >
T PolygonUtil::scalar_cross_product( const boost::numeric::ublas::vector< T > &a,
		const boost::numeric::ublas::vector< T > &b )
{
	assert( a.size() == 2 );
	assert( b.size() == 2 );
	return a(0) * b(1) - a(1) * b(0);
}



//Functions to calculate shortest path and edge visibility.


bool PolygonUtil::IsInsidePolygon(Point& p1, Point& p2, Polygon& polygon){

	Segment halfedge(p1,p2);

	Point intPoint;
	Segment intSegment;

	for (EdgeIterator ei = polygon.edges_begin(); ei != polygon.edges_end(); ++ei)
	{
		Object obj=CGAL::intersection(halfedge,*ei);
		if(CGAL::assign(intPoint,obj))
		{
			if(! (Equals(intPoint, p1) || Equals(intPoint,p2))  )
			{
				return false;
			}
		}
	}

	Point midPt((p1.cartesian(0)+p2.cartesian(0))/2, (p1.cartesian(1)+p2.cartesian(1))/2);
	return (CheckInside(midPt, polygon));
}


bool PolygonUtil::CheckInside(Point& pt,Polygon& polygon)
{
	bool flag = false;
	Point *pgn_begin = polygon.vertices_begin().base();
	Point *pgn_end = polygon.vertices_end().base();

	switch(CGAL::bounded_side_2(pgn_begin, pgn_end,pt, K())) {
	case CGAL::ON_BOUNDED_SIDE :
		flag = true;
		break;
	case CGAL::ON_BOUNDARY:
		flag = true;
		break;
	}
	return flag;
}


graph_t PolygonUtil::PrepareVisibilityGraph(Polygon& map){

	int n = map.size();
	Point vertex[n];

	int i = 0;
	for(VertexIterator vi = map.vertices_begin(); vi !=map.vertices_end(); ++vi){
		vertex[i] = *vi;
		i++;
	}

	std::list<tuple<int, int> > edgeList;

	for(int i = 0; i < n; i++){
		for(int j = i+1; j < n; j++){

			if(IsInsidePolygon(vertex[i], vertex[j], map)){
				tuple<int, int> t(i,j);
				edgeList.push_back(t);
			}
		}
	}

	std::list<tuple<int, int> >::iterator it;
	std::cout<<"Edge list is \n";

	int numEdges = edgeList.size();

	Edge edge_array[numEdges];
	int weights[numEdges];

	i = 0;
	for(it = edgeList.begin(); it != edgeList.end(); it++, i++){
		int i = get<0>(*it);
		int j = get<1>(*it);
		edge_array[i] = Edge(i,j);
		weights[i]    = CGAL::squared_distance(vertex[i],vertex[j]);
		std::cout << "wt of edge ..("<<i<<","<<j <<") is "<<weights[i] <<"\n";//////
	}

	graph_t g(edge_array, edge_array + numEdges, weights, n);
	property_map<graph_t, edge_weight_t>::type weightmap = get(edge_weight, g);

	std::vector<vertex_descriptor> p(num_vertices(g));
	std::vector<int> d(num_vertices(g));
	vertex_descriptor s = boost::vertex(0, g);

	char name[] = "ABCDEFGH";
//	name[0]= '0';
//	for(int i = 1; i < n; i++){
//		name[i] = name[i-1]++;
//		std::cout<<name[i]<<",";
//	}
	std::cout<<"--------\n";

	boost::dijkstra_shortest_paths(g, s, predecessor_map(&p[0]).distance_map(&d[0]));

//	std::cout << "distances and parents:" << std::endl;
//	graph_traits < graph_t >::vertex_iterator vi, vend;
//	for (tie(vi, vend) = vertices(g); vi != vend; ++vi) {
//		std::cout << "distance("  << ") = " << d[*vi] << ", ";
//		std::cout << "parent(" << name[*vi] << ") = " << name[p[*vi]] << std::
//				endl;
//	}
//	std::cout << std::endl;

	return g;
}
