/**
 * HypothesisGenerator.cpp
 *
 *  Created on: 14-Jan-2012
 *      Author: apurv, ashwani, prateek
 */

#include "HypothesisGenerator.h"
using namespace boost::numeric::ublas;

/**
 * Constructor
 * @params mapP: Map polygon
 * @params visP: Visibility polygon
 * @params pUtil: A object of polygon Util class
 * @params robotPos: Original Position of robot
 */


HypothesisGenerator::HypothesisGenerator(Polygon& mapP, Polygon& visP, Point& robotPos) {
	this->mapP = mapP;
	this->visP = visP;
	this->pUtil= PolygonUtil();
	this->robotPos = robotPos;

}

HypothesisGenerator::HypothesisGenerator(Polygon& mapP, Polygon& visP,Point& robotPos, PolygonUtil& pUtil){
	this->mapP = mapP;
	this->visP = visP;
	this->pUtil= pUtil;
	this->robotPos = robotPos;
}

HypothesisGenerator::HypothesisGenerator()
{

}
/**
 * Destructor
 */
HypothesisGenerator::~HypothesisGenerator() {

}
/**
 * @params ei: Edge Iterator of polygon
 * @params polygon: Polygon
 * @returns Segment which is next to the edge iterator.
 */
Segment HypothesisGenerator::GetNextEdge(EdgeIterator& ei, Polygon& polygon){
	if (ei == (polygon.edges_end()-1)){
		ei = polygon.edges_begin();
	}
	else{
		ei++;
	}
	return *ei;
}
/**
 *@returns list of hypothesis
 */

list<Point> HypothesisGenerator::GenHypothesis(){
	list<Point> hyps;

	EdgeIterator mapIter = mapP.edges_begin();
	EdgeIterator visIter = visP.edges_begin();

	EdgeIterator mapIterCpy = mapP.edges_begin();
	EdgeIterator visIterCpy = visP.edges_begin();

	Segment mapEdge = *(mapIter);
	Segment visEdge = *(visIter);

	for(int i = 0; i < mapP.size(); i++)
	{
		visIter = visP.edges_begin();
		visIterCpy = visP.edges_begin();
		for(int j = 0; j < visP.size(); j++)
		{
			if(IsMatch(mapEdge, visEdge))
			{
				Vector T(visEdge.point(0), mapEdge.point(0));
				Vector invT(mapEdge.point(0), visEdge.point(0));

				Transformation translate(CGAL::TRANSLATION, T);
				Transformation invTranslate(CGAL::TRANSLATION, invT);

				TranslatePolygon(translate, visP);
				robotPos = translate(robotPos);

				Polygon visibilityPolygon=pUtil.CalcVisibilityPolygon(mapP,robotPos);
				if(pUtil.doPolygonsMatch(visibilityPolygon,visP))
				{
					if(!IsInList(hyps,robotPos)){
						hyps.push_back(robotPos);
					}
				}

				TranslatePolygon(invTranslate, visP);
				robotPos = invTranslate(robotPos);
			}
			visEdge = GetNextEdge(visIter, visP);
			GetNextEdge(visIterCpy, visP);
		}
		mapEdge = GetNextEdge(mapIter, mapP);
		GetNextEdge(mapIterCpy, mapP);
	}
	return hyps;
}
/**
 *@params list<Point> hyps: list of hypothesis
 *@params p: Point
 *@returns a bool if p is present in the list.
 */
bool HypothesisGenerator::IsInList(list<Point> hyps, Point p){
	list<Point>::iterator it;
	for(it = hyps.begin(); it != hyps.end(); it++){
		if(pUtil.Equals(*it,p)){
			return true;
		}
	}
	return false;
}
/**
 * @params translate: Transformation object of CGAL
 * @params polygon: Polygon
 * translates the polygon by the translate object
 */

void HypothesisGenerator::TranslatePolygon(Transformation& translate, Polygon& polygon)
{
	for(VertexIterator vi = polygon.vertices_begin(); vi !=polygon.vertices_end(); ++vi)
	{
		*vi = translate(*vi);
	}
}

/**
 * Returns whether edge has atmost two vertices of polygon lying on this edge.
 * @param edge
 * @return
 */
bool HypothesisGenerator::TwoVertexOnEdge(Segment& edge){

	int count = 0;
	double slope=GetSlope(edge);
	for(VertexIterator vi = mapP.vertices_begin(); vi !=mapP.vertices_end(); ++vi){
		Point p = *vi;
		Segment seg(p,edge.point(0));
		if(edge.has_on(p) && !pUtil.EqualsValue(GetSlope(seg),slope))
		{
			count++;
		}
	}

	return (count <= 2);
}

/**
 * @params mapEdge: Segment of original map
 * @params visEdge: segment of visibility polygon
 * @returns if mapEdge matches visEdge
 */


bool HypothesisGenerator::IsMatch(Segment mapEdge, Segment visEdge){
	return (pUtil.EqualsValue(length(mapEdge),length(visEdge)) && pUtil.EqualsValue(orient(mapEdge),orient(visEdge)));
}

/**
 * @params edge: Segment
 * @returns slope of edge
 */
double HypothesisGenerator::GetSlope(Segment& edge){
	Point p2 = edge.point(1);
	Point p1 = edge.point(0);
	if(edge.is_vertical())
		return INFINITY;
	else
		return ( p2.cartesian(1) - p1.cartesian(1))/(p2.cartesian(0) - p1.cartesian(0));
}

/**
 * @params edge: Segment
 * @returns length of the edge
 */
double HypothesisGenerator::length(Segment edge){
	return edge.squared_length();
}
/**
 * @params edge: Segment
 * @returns the orientation of edge
 */
double HypothesisGenerator::orient(Segment edge){
	return GetSlope(edge);
}
