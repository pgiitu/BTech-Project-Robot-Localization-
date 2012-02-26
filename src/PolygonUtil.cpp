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
	if(check!=target && (CGAL::squared_distance(source,check)<=CGAL::squared_distance(source,target))
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
	pointList.clear();
	for(sit=pointSet.begin();sit!=pointSet.end();++sit)
	{
		pointList.push_back(*sit);
	}
	return pointList;
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


Polygon PolygonUtil::VisiblePointSet(Polygon& polygon, Point& point)
{
	Polygon result = Polygon();

	for (VertexIterator vi = polygon.vertices_begin(); vi != polygon.vertices_end(); ++vi)
	{

		Segment halfedge(point,*vi);
		Point intPoint;

		int flag=0;  //to check if corner point or not
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
std::list<Polygon> PolygonUtil::CalcAllVisibilityPolygon(Polygon& map,Polygon &P1,Polygon &P2)
{
	using namespace std;
	list<Polygon> polygonList;
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

	for(EdgeIterator ei=map.edges_begin();ei!=map.edges_end();++ei)
	{
		Segment seg(ei->source(),ei->target());
		if(ClassifyEdges(seg,P1,P2)==2)
		{
			Polygon visPolygon= CalcVisibilityPolygonEdge(map,seg,g,vertex);
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
    int zDirection=-1;

    int first=pathList.front();
    pathList.pop_front();
    int second=pathList.front();

    boost::numeric::ublas::vector<double> edgeVector(2);
    edgeVector(0)=vertex[second].cartesian(0)-vertex[first].cartesian(0);
    edgeVector(1)=vertex[second].cartesian(1)-vertex[first].cartesian(1);

    std::cout<<"Segment Vector  "<<seg(0)<<"  "<<seg(1)<<endl;
    double crossProd=scalar_cross_product(seg,edgeVector);
    double dotProd=dot_product(seg,edgeVector);

    float initialAngle=acos(dotProd/(sqrt(pow(edgeVector(0),2)+pow(edgeVector(1),2))*sqrt(pow(seg(0),2)+pow(seg(1),2))));

    if (crossProd <= 0)
    {
    	flag=1; //setting the flag
    	initialAngle=2*M_PI-initialAngle;
    }

    std::cout<<"Initial angle"<<initialAngle;
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

        std::cout<<"Angle " <<angle<<" Flag  "<<flag;
 //       cout<<"Angle  "<<angle<<" Flag "<<flag<<endl;
        if(flag and angle> initialAngle and fabs(initialAngle-angle)>E)
        {
        	std::cout<<"Correct \n";
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

	std::list<Point> visibilityPolygonList;

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
			visibilityPolygonList.push_back((*ei).source());
			visibilityPolygonList.push_back((*ei).target());
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

			cout<<"Edge Source   "<<edgeSource <<" Edge Target  "<<edgeTarget<<endl;
			cout<<"source to source "<<endl;
			bool convex1=isPathConvex(vertex,segSourceToedgeSourceList,sourceTarget);
			cout<< " Path Convex  "<<convex1;

			printList(segSourceToedgeSourceList);


			cout<<"target to target "<<endl;
			bool convex2=isPathConvex(vertex,segTargetToedgeTargetList,targetSource);
			cout<<" Path convex  "<<convex2<<endl;
			printList(segTargetToedgeTargetList);

			cout<<"Source to target "<<endl;
			bool convex3=isPathConvex(vertex,segSourceToedgeTargetList,sourceTarget);
			cout<< " Path Convex  "<<convex3<<endl;
			printList(segSourceToedgeTargetList);


			cout<<"Target to source "<<endl;
			bool convex4=isPathConvex(vertex,segTargetToedgeSourceList,targetSource);
			cout<<" Path Convex  "<<convex4<<endl;
			printList(segTargetToedgeSourceList);

			if(convex1  && convex2	&& !isPathIntersect(vertex,segSourceToedgeSourceList,segTargetToedgeTargetList) )
			{
				cout<<"Ashwani \n";
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
				cout<<"1Cusp "<<cusp1<<endl;
				cout<<"1AdjacentCusp "<<adjacentCusp1<<endl;
				cout<<"2Cusp "<<cusp2<<endl;
				cout<<"2AdjacentCusp "<<adjacentCusp2<<endl;
			}



			else if(convex3 && convex4 && !isPathIntersect(vertex,segSourceToedgeTargetList,segTargetToedgeSourceList))
			{
				cout<<"Garg"<<endl;
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

				cout<<"Cusp1   "<<cusp1<<"  adjacent cusp1  "<<adjacentCusp1<<endl;
				cout<<"Cusp2   "<<cusp2<<"  adjacent cusp2  "<<adjacentCusp2<<endl;

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
						cout<<"Inserted Point "<<insertPoint<<endl;

						visibilityPolygonList.push_back(insertPoint);
					//}
				}
				else if(CGAL::assign(intSeg,obj1))
				{
					//cout<<" Inserted Point "<<intSeg.target();
					visibilityPolygonList.push_back(intSeg.target());
					visibilityPolygonList.push_back(intSeg.source());
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

				//	if(!Equals(insertPoint,edgeSource))
				//	{
						cout<<"Inserted Point "<<insertPoint<<endl;
						visibilityPolygonList.push_back(insertPoint);
				//	}
				}
				else if(CGAL::assign(intSeg,obj2))
				{
					//cout<<" Inserted Point "<<intSeg.target();
					visibilityPolygonList.push_back(intSeg.target());
					visibilityPolygonList.push_back(intSeg.source());
				}

			}

		}
	}

	visibilityPolygonList=UniqueList(visibilityPolygonList);
	std::list<Point>::iterator listIt;

	for(listIt=visibilityPolygonList.begin();listIt!=visibilityPolygonList.end();++listIt)
	{
		setVisiblePoints.push_back(*listIt);
	}

	setVisiblePoints=SortPolygon(setVisiblePoints,map);

	return setVisiblePoints;

}


graph_t PolygonUtil::PrepareVisibilityGraph(Polygon& map, Point vertex[]){

	int n = map.size();
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
				result=result+1;
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
				result=result+1;
	}

	return result;

}
Polygon PolygonUtil::CalcGPolygon(Point& center,Polygon& FPolygon,std::list<Polygon> polygonList)
{
	using namespace std;
	Polygon GPolygon;

	list<Polygon> visibilityPolygonList;

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
	/*
	 * In this overlay arrangement find the region containing origin
	 * return that polygon
	 *
	 */

	return GPolygon;

}
