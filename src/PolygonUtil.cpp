/*
 * PolygonUtil.cpp
 *
 *  Created on: 13-Jan-2012
 *      Author: apurv
 */

#include "PolygonUtil.h"
#include "Majoritymap.h"
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
#include <boost/graph/bellman_ford_shortest_paths.hpp>

#define E 0.00001
using namespace boost;

Point fixedPoint;

bool CompareDistance(Point p1,Point p2);
bool CompareDistance1(Point p1,Point p2);

void printList(std::list<int> list1);

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
//			Segment s(rayToCorner.point(0),intPoint);
			if((intPoint != vertexPolygon) && (!IsVertexOfPolygon(map, intPoint))
					&& (IsInsidePolygon1(vertexPolygon,intPoint,map)))
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
	/*
	 * changes have been done, It may harm the functionality of visibility Polygon  of a Point
	 */
	using namespace std;
	Polygon setVisiblePoints = VisiblePointSet(map,point);
//	cout<<"Set Of Visible Points\n";
//	DisplayPolygon(setVisiblePoints);

	list<Point> listVisiblePoints;
	fixedPoint=point;

	for (VertexIterator vi = setVisiblePoints.vertices_begin(); vi != setVisiblePoints.vertices_end(); ++vi)
	{
		listVisiblePoints.push_back(*vi);
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
				intPointList.sort(CompareDistance1);
				intPointList.unique();
				Point spuriousVertex = *intPointList.begin();
				p2=spuriousVertex;
				listVisiblePoints.push_back(spuriousVertex);
			}
		}

	}

	listVisiblePoints=UniqueList(listVisiblePoints);
	list<Point>::iterator it;
	Polygon setVisiblePointsCopy;
	for(it=listVisiblePoints.begin();it!=listVisiblePoints.end();++it)
	{
//		cout<<"Visibility Polygon Point"<<*it<<"\n";
		setVisiblePointsCopy.push_back(*it);
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
	if((CGAL::squared_distance(source,check)<=CGAL::squared_distance(source,target))
			&& (CGAL::squared_distance(target,check)<=CGAL::squared_distance(source,target)))
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


std::list<Point> PolygonUtil::UniqueList(std::list<Point> pointList)
{
	std::set<Point> pointSet;
	std::list<Point>::iterator it;
	for(it=pointList.begin();it!=pointList.end();++it)
	{
		pointSet.insert(*it);
	}
	std::set<Point>::iterator sit;

	std::list<Point> pointListCopy;
	pointList.clear();
	for(sit=pointSet.begin();sit!=pointSet.end();++sit)
	{
		pointListCopy.push_back(*sit);
	}
	return pointListCopy;
}

/*
 *
 */
bool PolygonUtil::isPointInList(std::list<Point> pointList,Point p)
{
	std::list<Point>::iterator it;
	for(it=pointList.begin();it!=pointList.end();++it)
	{
		if(Equals(*it,p))
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
	std::list<Point> visibilyPolygonSet;

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
		UniqueList(PointList);
		PointList.sort(CompareDistance1);

		for(it=PointList.begin();it!=PointList.end();++it)
		{
			if(!isPointInList(visibilyPolygonSet,*it))
				visibilyPolygonSet.push_back(*it);
		}

	}

	visibilyPolygonSet.unique();
	Polygon visibilyPolygon;
	std::list<Point>::iterator sit;
	for(sit=visibilyPolygonSet.begin();sit!=visibilyPolygonSet.end();++sit)
	{
		visibilyPolygon.push_back(*sit);
	}
	return visibilyPolygon;
}




void PolygonUtil::insertBefore(Polygon& polygon, VertexIterator& vi, Point& spurPoint){

	polygon.insert(vi, spurPoint);
	vi--;
}


Polygon PolygonUtil::VisiblePointSet(Polygon& polygon, Point& point)
{
	Polygon result = Polygon();
	for (VertexIterator vi = polygon.vertices_begin(); vi != polygon.vertices_end(); ++vi)
	{

		Segment halfedge(point,*vi);
		Point intPoint;
		Point p(6.5,0);
/*
		if(Equals(*vi, p))
			std::cout<<IsInsidePolygon(point,*vi,polygon)<<"\n\n";
*/


		if(IsInsidePolygon1(point,*vi,polygon))
		{

			result.push_back(*vi);
		}
	/*	int flag=0;  //to check if corner point or not
		for (EdgeIterator ei = polygon.edges_begin(); ei != polygon.edges_end(); ++ei)
		{

			Object obj=CGAL::intersection(halfedge,*ei);
			if(CGAL::assign(intPoint,obj))
			{

				if( !IsVertexOfPolygon(polygon, intPoint))
				{
					flag=1;
					break;
				}
			}
		}
		if(flag==0)
		{
			result.push_back(*vi);
		}
*/	}
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
	Point robotPos(6,0);  //point from where we are calculating the visibility polygon

	if(CGAL::squared_distance(robotPos,p1)<CGAL::squared_distance(robotPos,p2))
		return true;
	else
		return false;
}

bool PolygonUtil::EqualsValue(double v1, double v2)
{
	return  (fabs(v1 - v2)<E) ;

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

template< typename T >
T PolygonUtil::dot_product( const boost::numeric::ublas::vector< T > &a,
		const boost::numeric::ublas::vector< T > &b )
{
	assert( a.size() == 2 );
	assert( b.size() == 2 );
	return a(0) * b(0) + a(1) * b(1);
}


//Functions to calculate shortest path and edge visibility.


bool PolygonUtil::IsInsidePolygon(Point& p1, Point& p2, Polygon& polygon){

	Segment halfedge(p1,p2);

	Point intPoint;
	Segment intSegment;

	Point p(6.5,0);
	for (EdgeIterator ei = polygon.edges_begin(); ei != polygon.edges_end(); ++ei)
	{
		Object obj=CGAL::intersection(halfedge,*ei);
		if(CGAL::assign(intPoint,obj))
		{
			if(! (Equals(intPoint, p1) || Equals(intPoint,p2)))
			{
				if(Equals(p2,p))
					cout<<(*ei)<<" Point On Segment  "<<IsPointOnSegment(p1,p2,intPoint)<<intPoint<<"    Return false\n";
				return false;
			}
		}
	}

	Point midPt((p1.cartesian(0)+p2.cartesian(0))/2, (p1.cartesian(1)+p2.cartesian(1))/2);
	if(Equals(p2,p))
		cout<<p1<<"  "<<p2<<"  "<<CheckInside(midPt,polygon)<<"\n\n";
	return (CheckInside(midPt, polygon));
}

/*
 * By checking the mid points
 */
bool PolygonUtil::IsInsidePolygon1(Point& p1, Point& p2, Polygon& polygon)
{
	std::list<Point> listOfPoints;
	Segment halfedge(p1,p2);

	Point intPoint;
	Segment intSegment;

	for (EdgeIterator ei = polygon.edges_begin(); ei != polygon.edges_end(); ++ei)
	{
		Object obj=CGAL::intersection(halfedge,*ei);
		if(CGAL::assign(intPoint,obj))
		{
			listOfPoints.push_back(intPoint);
		}
	}

	listOfPoints=UniqueList(listOfPoints);
	fixedPoint=p1;
	listOfPoints.sort(CompareDistance1);

	std::list<Point>::iterator pit;
	Point firstPoint=listOfPoints.front();
	listOfPoints.pop_front();

	for(pit=listOfPoints.begin();pit!=listOfPoints.end();++pit)
	{
		Point midPoint((firstPoint.cartesian(0)+(*pit).cartesian(0))/2,(firstPoint.cartesian(1)+(*pit).cartesian(1))/2);
		if(!CheckInside(midPoint,polygon))
			return false;
		firstPoint=*pit;
	}

	return true;
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

Polygon PolygonUtil::CalcWindowEdge(Polygon& map, Segment seg)
{

/*
 * Maps the polygon
 */
	int n = map.size();
	Point vertex[n];
	int i = 0;

	for(VertexIterator vi = map.vertices_begin(); vi !=map.vertices_end(); ++vi){
		vertex[i] = *vi;
		i++;
	}

	//visibility Graph
	graph_t g=PrepareVisibilityGraph(map,vertex);

	//Segment whose visibility polygon needs to be drawn

//	for(EdgeIterator ei=map.edges_begin();ei!=map.edges_end();++ei)
//	{
		//Segment seg(Point(0,-1),Point(-1,-1));
		Polygon visPolygon= CalcVisibilityPolygonEdge(map,seg,g,vertex);
	    return visPolygon;
}
std::list<Polygon> PolygonUtil::CalcAllVisibilityPolygon(Polygon& Fij,Polygon &P1,Polygon &P2)
{
	using namespace std;
	list<Polygon> polygonList;
/*
 * Maps the polygon
 */
	int n = Fij.size();
	Point vertex[n];
	int i = 0;

	for(VertexIterator vi = Fij.vertices_begin(); vi !=Fij.vertices_end(); ++vi){
		vertex[i] = *vi;
		i++;
	}

	//visibility Graph
	graph_t g=PrepareVisibilityGraph(Fij,vertex);

	//Segment whose visibility polygon needs to be drawn

	for(EdgeIterator ei=Fij.edges_begin();ei!=Fij.edges_end();++ei)
	{
		Segment seg(ei->source(),ei->target());
		if(ClassifyEdges(seg,P1,P2)==2)
		{

//			cout<<"Segment is "<<seg<<endl;
			Polygon visPolygon= CalcVisibilityPolygonEdge(Fij,seg,g,vertex);
//			cout<<"Polygon is";
//			DisplayPolygon(visPolygon);
			polygonList.push_back(visPolygon);
		}

	}
	//Calculate the visibility polygon


	return polygonList;
}


int PolygonUtil::findIndex(Point vertex[],int size,Point point)
{
	for(int i=0;i<size;i++)
	{
		if(Equals(vertex[i],point))
			return i;
	}

	return -1;
}

std::vector<vertex_descriptor> PolygonUtil::CalcShortestPath(int source,graph_t& g,Point vertex[])
{
	  property_map<graph_t, edge_weight_t>::type weightmap = get(edge_weight, g);
	  std::vector<vertex_descriptor> p(num_vertices(g));
	  std::vector<float> d(num_vertices(g));

	  vertex_descriptor s = boost::vertex(source, g);

	  dijkstra_shortest_paths(g, s, predecessor_map(&p[0]).distance_map(&d[0]));

//	  std::cout << "distances and parents:" << std::endl;
	  graph_traits < graph_t >::vertex_iterator vi, vend;
/*
	  for (tie(vi, vend) = vertices(g); vi != vend; ++vi) {
	    std::cout << "distance(" << *vi << ") = " << d[*vi] << ", ";
	    std::cout << "parent(" << *vi << ") = " << p[*vi] << std::
	      endl;
	  }
	  std::cout << std::endl;
*/

	  return p;
}
/*
 * returns a pat from source to target
 */

int PolygonUtil::path(std::vector<vertex_descriptor> pathVector,int source,int target,std::list<int>& pathList){
	int i=0;
	pathList.push_front(target);
	while(pathVector[target]!=source)
	{
		pathList.push_front(pathVector[target]);
		target=pathVector[target];
	}

	pathList.push_front(source);
	return i;
}

bool PolygonUtil::isPathConvex(Point vertex[],std::list<int> pathList,boost::numeric::ublas::vector<double> seg)
{

	//Take Care Of Angles
	bool flag=0;

    int first=pathList.front();
    pathList.pop_front();
    int second=pathList.front();

    boost::numeric::ublas::vector<double> edgeVector(2);
    edgeVector(0)=vertex[second].cartesian(0)-vertex[first].cartesian(0);
    edgeVector(1)=vertex[second].cartesian(1)-vertex[first].cartesian(1);

//    std::cout<<"Segment Vector  "<<seg(0)<<"  "<<seg(1)<<endl;
    double crossProd=scalar_cross_product(seg,edgeVector);
    double dotProd=dot_product(seg,edgeVector);

    float initialAngle=acos(dotProd/(sqrt(pow(edgeVector(0),2)+pow(edgeVector(1),2))*sqrt(pow(seg(0),2)+pow(seg(1),2))));

    if (crossProd <= 0)
    {
    	flag=1; //setting the flag
    	initialAngle=2*M_PI-initialAngle;
    }

//    std::cout<<"Initial angle"<<initialAngle;
    std::list<int>::iterator it;
    pathList.pop_front();
    for(it=pathList.begin();it!=pathList.end();++it)
    {
    	first=second;
    	second=*it;
        edgeVector(0)=vertex[second].cartesian(0)-vertex[first].cartesian(0);
        edgeVector(1)=vertex[second].cartesian(1)-vertex[first].cartesian(1);
        crossProd=scalar_cross_product(seg,edgeVector);
        dotProd=dot_product(seg,edgeVector);
/*
        if(crossProd < 0)   //angle becomes concave
        	return false;
*/

 //       std::cout<<"Edge Vector  "<<edgeVector(0)<<"  "<<edgeVector(1)<<endl;

        float angle=acos(dotProd/(sqrt(pow(edgeVector(0),2)+pow(edgeVector(1),2))*sqrt(pow(seg(0),2)+pow(seg(1),2))));

        if (crossProd<0)
            	angle=2*M_PI-angle;

//        std::cout<<"Angle " <<angle<<" Flag  "<<flag;
 //       cout<<"Angle  "<<angle<<" Flag "<<flag<<endl;
        if(flag and angle> initialAngle and fabs(initialAngle-angle)>E)
        {
//        	std::cout<<"Correct \n";
        	return false;
        }
        if(!flag and angle <initialAngle and fabs(initialAngle-angle)>E)
        	return false;


        initialAngle=angle;
    }

	return true;
}

bool PolygonUtil::isPathIntersect(Point vertex[],std::list<int> path1,std::list<int> path2)
{
	bool result;
	Segment edgeArray1 [path1.size()-1];
	Segment edgeArray2 [path2.size()-1];

	std::list<int>::iterator it;

	int i=0;
	Point first(vertex[path1.front()]);
	path1.pop_front();
//	std::cout<<" Path1 \n";
	for(it=path1.begin();it!=path1.end();++it)
	{
		edgeArray1[i++]=Segment(vertex[*it],first);
		//std::cout<< " edge  "<<edgeArray1[i++]<<endl;
		first=vertex[*it];

	}

/*
	for(int k=0;k<=(path1.size()-1);k++)
	{
		std::cout<<edgeArray1[k]<<std::endl;
	}
*/


	i=0;
	first= vertex[path2.front()];
	path2.pop_front();
//	std::cout<< "Path 2 \n";
	for(it=path2.begin();it!=path2.end();++it)
	{
		edgeArray2[i++]=Segment(vertex[*it],first);
	//	std::cout<< " edge  "<<edgeArray2[i++]<<endl;
		first=vertex[*it];
	}

/*
	for(int k=0;k<=(path2.size()-1);k++)
	{
		std::cout<<edgeArray2[k]<<std::endl;
	}
*/

	for(i=0;i<=(path1.size()-1);i++)
	{
//		std::cout<<" edge1 " <<edgeArray1[i] <<"\n";
		for(int j=0;j<=(path2.size()-1);j++)
		{

//			std::cout<<" Edge2 " <<edgeArray2[j] <<"\n";
			if(CGAL::do_intersect(edgeArray1[i],edgeArray2[j]))
				return true;
		}
	}

	return false;

}
int PolygonUtil::findCommonAncestor(std::list<int> path1,std::list<int> path2)
{
	int value=-1;

	std::list<int>::iterator it2;
	std::list<int>::iterator it1;

	path2.reverse();
	path2.pop_front();

	for(it2=path2.begin();it2!=path2.end();++it2)
	{
		for(it1=path1.begin();it1!=path1.end();++it1)
		{
			if((*it1)==(*it2))
			{
				value=*it1;
				return value;
			}
		}
	}
	return value;
}


void printList(std::list<int> list1)
{
	std::list<int>::iterator it;
	for(it=list1.begin();it!=list1.end();++it)
	{
		std::cout<<"    "<<*it;
	}
	std::cout<< "\n";
}

Polygon PolygonUtil::CalcVisibilityPolygonEdge(Polygon& map, Segment& seg, graph_t& g, Point vertex[])
{
	using namespace std;
	Polygon setVisiblePoints;

	std::set<Point> set;

	Point segSource(seg.source());
	Point segTarget(seg.target());

	int segSourceIndex=findIndex(vertex,map.size(),segSource);
	int segTargetIndex=findIndex(vertex,map.size(),segTarget);


	std::vector<vertex_descriptor> sourcePath=CalcShortestPath(segSourceIndex,g,vertex);
	std::vector<vertex_descriptor> targetPath=CalcShortestPath(segTargetIndex,g,vertex);

	boost::numeric::ublas::vector<double> sourceTarget(2);//(segSource,segTarget);
	boost::numeric::ublas::vector<double> targetSource(2);//(segTarget,segSource);

	sourceTarget(0)=segTarget.cartesian(0)-segSource.cartesian(0);
	sourceTarget(1)=segTarget.cartesian(1)-segSource.cartesian(1);

	targetSource(0)=segSource.cartesian(0)-segTarget.cartesian(0);
	targetSource(1)=segSource.cartesian(1)-segTarget.cartesian(1);

	for (EdgeIterator ei = map.edges_begin(); ei != map.edges_end(); ++ei)
	{


		Segment edge((*ei).source(),(*ei).target());
		if(edge==seg)
		{

			set.insert((*ei).source());
			set.insert((*ei).target());
		}

		else
		{
			Point edgeSource(edge.source());
			Point edgeTarget(edge.target());
			int edgeSourceIndex=findIndex(vertex,map.size(),edgeSource);
			int edgeTargetIndex=findIndex(vertex,map.size(),edgeTarget);

			std::list<int> segSourceToedgeSourceList;
			std::list<int> segTargetToedgeTargetList;

			std::list<int> segSourceToedgeTargetList;
			std::list<int> segTargetToedgeSourceList;

			path(sourcePath,segSourceIndex,edgeSourceIndex,segSourceToedgeSourceList);
			path(targetPath,segTargetIndex,edgeTargetIndex,segTargetToedgeTargetList);
			path(sourcePath,segSourceIndex,edgeTargetIndex,segSourceToedgeTargetList);
			path(targetPath,segTargetIndex,edgeSourceIndex,segTargetToedgeSourceList);

			int cusp1,adjacentCusp1,cusp2,adjacentCusp2;
			bool flag=false;

//			cout<<"Edge Source   "<<edgeSource <<" Edge Target  "<<edgeTarget<<endl;
//			cout<<"source to source "<<endl;
			bool convex1=isPathConvex(vertex,segSourceToedgeSourceList,sourceTarget);
//			cout<< " Path Convex  "<<convex1;

//			printList(segSourceToedgeSourceList);


//			cout<<"target to target "<<endl;
			bool convex2=isPathConvex(vertex,segTargetToedgeTargetList,targetSource);
//			cout<<" Path convex  "<<convex2<<endl;
//			printList(segTargetToedgeTargetList);
//
//			cout<<"Source to target "<<endl;
			bool convex3=isPathConvex(vertex,segSourceToedgeTargetList,sourceTarget);
//			cout<< " Path Convex  "<<convex3<<endl;
//			printList(segSourceToedgeTargetList);


//			cout<<"Target to source "<<endl;
			bool convex4=isPathConvex(vertex,segTargetToedgeSourceList,targetSource);
//			cout<<" Path Convex  "<<convex4<<endl;
//			printList(segTargetToedgeSourceList);

			if(convex1  && convex2	&& !isPathIntersect(vertex,segSourceToedgeSourceList,segTargetToedgeTargetList) )
			{
				cusp1=findCommonAncestor(segSourceToedgeSourceList,segSourceToedgeTargetList);

				flag=true;
				std::list<int>::iterator it;
				bool common=false;

				for(it=segSourceToedgeTargetList.begin();it!=segSourceToedgeTargetList.end();++it)
				{
					if((*it)==cusp1)
						break;
				}

				++it;
				adjacentCusp1=*it;

				cusp2=findCommonAncestor(segTargetToedgeSourceList,segTargetToedgeTargetList);

				for(it=segTargetToedgeSourceList.begin();it!=segTargetToedgeSourceList.end();++it)
					{
						if((*it)==cusp2)
						{
							break;
						}
					}
	//			cout<<" After Loop "<<*it<<"List End " <<*(segTargetToedgeSourceList.end()-1)<<endl;
				if(it!=(--segTargetToedgeSourceList.end()))
					{
						++it;
						adjacentCusp2=*it;
					}
				else
					{
						adjacentCusp2=cusp2;
					}
//				cout<<"1Cusp "<<cusp1<<endl;
//				cout<<"1AdjacentCusp "<<adjacentCusp1<<endl;
//				cout<<"2Cusp "<<cusp2<<endl;
//				cout<<"2AdjacentCusp "<<adjacentCusp2<<endl;
			}



			else if(convex3 && convex4 && !isPathIntersect(vertex,segSourceToedgeTargetList,segTargetToedgeSourceList))
			{
				cusp1=findCommonAncestor(segSourceToedgeSourceList,segSourceToedgeTargetList);
				flag=true;
				std::list<int>::iterator it;

				for(it=segSourceToedgeSourceList.begin();it!=segSourceToedgeSourceList.end();++it)
					{
						if((*it)==cusp1)
							break;
					}
				++it;
				adjacentCusp1=*it;

			   cusp2=findCommonAncestor(segTargetToedgeSourceList,segTargetToedgeTargetList);

			   for(it=segTargetToedgeTargetList.begin();it!=segTargetToedgeTargetList.end();++it)
					{
				   	   if((*it)==cusp2)
						{
							break;
						}

					}

				if(it!=--segTargetToedgeTargetList.end())
					{
					++it;
					adjacentCusp2=*it;
					}
				else
					adjacentCusp2=cusp2;

//				cout<<"Cusp1   "<<cusp1<<"  adjacent cusp1  "<<adjacentCusp1<<endl;
//				cout<<"Cusp2   "<<cusp2<<"  adjacent cusp2  "<<adjacentCusp2<<endl;

			}


			if(flag)
			{
				Ray extend1(vertex[cusp1],vertex[adjacentCusp1]);
				Ray extend2(vertex[cusp2],vertex[adjacentCusp2]);

				Object obj1=CGAL::intersection(extend1,*ei);
				Object obj2=CGAL::intersection(extend2,*ei);

				Point intPoint;
				Segment intSeg;

				if(CGAL::assign(intPoint,obj1))
				{
					double x=intPoint.cartesian(0);
					double y=intPoint.cartesian(1);

					if(fabs(x) <E)
						x=0;
					if(fabs(y) <E)
						y=0;

					Point insertPoint(x,y);
					//if(!Equals(insertPoint,edgeSource))
					//{
//						cout<<"Inserted Point "<<insertPoint<<endl;

						set.insert(insertPoint);
					//}
				}
				else if(CGAL::assign(intSeg,obj1))
				{
					//cout<<" Inserted Point "<<intSeg.target();
					set.insert(intSeg.target());
					set.insert(intSeg.source());

				}

				if(CGAL::assign(intPoint,obj2))
				{
					double x=intPoint.cartesian(0);
					double y=intPoint.cartesian(1);

					if(fabs(x) <E)
						x=0;
					if(fabs(y) <E)
						y=0;

					Point insertPoint(x,y);
//						cout<<"Inserted Point "<<insertPoint<<endl;
						set.insert(insertPoint);
				}
				else if(CGAL::assign(intSeg,obj2))
				{
					set.insert(intSeg.target());
					set.insert(intSeg.source());
				}
			}
		}
	}

	std::set<Point>::iterator setIt;
	Polygon setVisible;
	for(setIt=set.begin();setIt!=set.end();++setIt)
	{
		setVisible.push_back(*setIt);
	}

	setVisible=SortPolygon(setVisible,map);
	return setVisible;
}


graph_t PolygonUtil::PrepareVisibilityGraph(Polygon& map, Point vertex[]){

	int n = map.size();
	std::list<tuple<int, int> > edgeList;

	for(int i = 0; i < n; i++){
		for(int j = i+1; j < n; j++){
/*
 * The function IsinsidePolygon may have been changed
 */
			if(IsInsidePolygon(vertex[i], vertex[j], map)){
				tuple<int, int> t(i,j);
				edgeList.push_back(t);
			}
		}
	}

	std::list<tuple<int, int> >::iterator it;
	int numEdges = edgeList.size();

	// Initialinzing the edges and weights array to form the dual graph
	Edge edge_array[numEdges];
	float weights[numEdges];

	int k = 0;
	for(it = edgeList.begin(); it != edgeList.end(); it++, k++){
		int i = get<0>(*it);
		int j = get<1>(*it);
		edge_array[k] = Edge(i,j);
		weights[k]    =sqrt(CGAL::squared_distance(vertex[i],vertex[j]));
	}


		const int num_nodes = n;

		graph_t g(edge_array, edge_array + numEdges, weights, num_nodes);
		return g;
}

int PolygonUtil::ClassifyEdges(Segment edge, Polygon& P1,Polygon& P2)
{
	int result=1;
		/*
		 * Iterate over all the edges of P1 and see if edge is subset of any
		 * edge of P1
		 *
		 * Call another function to check if a point lies on edge or not
		 */


	for(EdgeIterator ei=P1.edges_begin();ei!=P1.edges_end();++ei)
	{
			Point source=ei->source();
			Point target=ei->target();
			if(IsPointOnSegment(source,target,edge.source()) &&
				IsPointOnSegment(source,target,edge.target()))
			{
				result=result+1;
				break;
			}
	}
	/*
	 *Iterate over all the edges of P2 and see if edge is subset of any
	 * edge of P2
	 */

	for(EdgeIterator ei=P2.edges_begin();ei!=P2.edges_end();++ei)
	{
			Point source=ei->source();
			Point target=ei->target();
			if(IsPointOnSegment(source,target,edge.source()) &&
				IsPointOnSegment(source,target,edge.target()))
			{
				result=result+1;
				break;
			}
	}

	return result;

}

Polygon PolygonUtil::CalcGPolygon(Point& center,Polygon& FPolygon,std::list<Polygon> polygonList)
{
	using namespace std;
	Polygon GPolygon;

	list<Polygon> visibilityPolygonList;

	/*
	 * only two polygons are there
	 */
    Polygon P1=polygonList.front();
    Polygon P2=polygonList.back();

	Majoritymap overlay;

	/*
	 * Iterate over all the edges of FPolygon and classify the edges
	 * Call a function classifyEdges(Segment edge,Polygon &P1,Polygon &P2)
	 */

	visibilityPolygonList=CalcAllVisibilityPolygon(FPolygon,P1,P2);

	//Push FPolygon also

	visibilityPolygonList.push_back(FPolygon);
	/*
	 * List contains all the visibility polygons of edges of type 1 and 2 and FPolygon
	 * Make an overlay arrangement corresponding to these polygons
	 *
	 */

	overlay.GenerateOverlay(visibilityPolygonList);

	GPolygon=overlay.OverlayContaningOrigin(center);
//	DisplayPolygon(GPolygon);

	/*
	 * In this overlay arrangement find the region containing origin
	 * return that polygon
	 *
	 */

	return GPolygon;

}

VertexIterator GetNextVertex(VertexIterator &vi,Polygon &P)
{
	if (vi == (P.vertices_end()-1)){
		vi = P.vertices_begin();
	}
	else{
		vi++;
	}
	return vi;
}


Polygon PolygonUtil::RemoveCollinearPoints(Polygon P)
{
	Polygon polygon;
	EdgeIterator ei=P.edges_begin();
//	polygon.push_back((*ei).point(0));
	Segment firstSegment((*ei).point(0),(*ei).point(1));
	double firstSlope=GetSlope(firstSegment);
	ei++;
	for(;ei!=P.edges_end();++ei)
	{
		Segment nextSegment((*ei).point(0),(*ei).point(1));
		double nextSlope=GetSlope(nextSegment);
		if(!EqualsValue(firstSlope,nextSlope))
		{
			polygon.push_back(nextSegment.point(0));
			firstSlope=nextSlope;
		}
	}

	ei=P.edges_begin();
	Segment frontSegment((*ei).point(0),(*ei).point(1));
	firstSlope=GetSlope(frontSegment);
	EdgeIterator lei=P.edges_end()-1;
	Segment endSegment((*lei).point(0),(*lei).point(1));
	double lastSlope=GetSlope(endSegment);
	if(!EqualsValue(firstSlope,lastSlope))
		polygon.push_back(frontSegment.point(0));


	return polygon;
}


double PolygonUtil::GetSlope(Segment& edge){
	Point p2 = edge.point(1);
	Point p1 = edge.point(0);
	if(edge.is_vertical())
		return INFINITY;
	else
		return ( p2.cartesian(1) - p1.cartesian(1))/(p2.cartesian(0) - p1.cartesian(0));
}


bool PolygonUtil::doPolygonsMatch(Polygon &P1copy,Polygon &P2copy)
{
	Polygon P1=RemoveCollinearPoints(P1copy);
	Polygon P2=RemoveCollinearPoints(P2copy);


	cout<<"Polygon P1 \n";
	DisplayPolygon(P1);
	cout<<"Polygon P2 \n";
	DisplayPolygon(P2);


	if(P1.size()!=P2.size())
		return false;

	VertexIterator v1,v2;
	v1=P1.vertices_begin();

	for(v2=P2.vertices_begin();v2!=P2.vertices_end();++v2)
		{
			if(Equals(*v1,*v2))
			{
				break;
			}
		}

//	VertexIterator v1copy=GetNextVertex(v1,P1);
//	VertexIterator v2copy=GetNextVertex(v2,P2);
	int n=P1.size();
	int i=0;
	while(i<n)
	{
		if(!Equals(*v1,*v2))
		{
			std::cout<<"False returned  "<<*v1<<"     "<<*v2<<"\n";
			DisplayPolygon(P2);
			return false;
		}

		GetNextVertex(v1,P1);
		GetNextVertex(v2,P2);
		i++;
	}
	return true;
}
