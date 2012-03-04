/*
 * HypothesisGenerator.h
 *
 *  Created on: 14-Jan-2012
 *      Author: apurv
 */
#include "constants.h"
#include "PolygonUtil.h"

#include <map>
#include <list>
#include <utility>

#ifndef HYPOTHESISGENERATOR_H_
#define HYPOTHESISGENERATOR_H_

using namespace std;

class cmp_segments{
public:
	bool operator()(Segment s1, Segment s2);
};

class HypothesisGenerator {

private:
	Polygon mapP;
	Polygon visP;
	PolygonUtil pUtil;
	Point robotPos;

	Segment GetNextEdge(EdgeIterator& ei, Polygon& polygon);
	double GetSlope(Segment& edge);


	double length(Segment edge);//Returns the square of length of an edge.
	double orient(Segment edge);


public:

	HypothesisGenerator();
	HypothesisGenerator(Polygon& mapP, Polygon& visP, Point& robotPos);
	HypothesisGenerator(Polygon& mapP, Polygon& visP, Point& robotPos, PolygonUtil& pUtil);

	virtual ~HypothesisGenerator();

	list<Point> GenHypothesis();
	void TranslatePolygon(Transformation& translate, Polygon& polygon);

	bool IsMatch(Segment mapEdge, Segment visEdge);
	bool IsComplMatch( EdgeIterator& visIter);
	bool TwoVertexOnEdge(Segment& edge);
	bool IsInList(list<Point> hyps, Point p);
};

#endif /* HYPOTHESISGENERATOR_H_ */
