/**
 * HypothesisGenerator.h
 *
 *  Created on: 14-Jan-2012
 *  Author: Apurv, Ashwani, Prateek
 */
#include "constants.h"
#include "PolygonUtil.h"

#include <map>
#include <list>
#include <utility>

#ifndef HYPOTHESISGENERATOR_H_
#define HYPOTHESISGENERATOR_H_

using namespace std;
/**
 * @params s1: Represents the segment
 * @params s2: Represents the segment
 * @returns bool indicating if s1 equals s2
 */
class cmp_segments{
public:
	bool operator()(Segment s1, Segment s2);
};

class HypothesisGenerator {

private:
	/**
	 * Class members
	 * @params mapP: Map polygon
	 * @params visP: Visibility polygon
	 * @params pUtil: A object of polygon Util class
	 * @params robotPos: Original Position of robot
	 */
	Polygon mapP;
	Polygon visP;
	PolygonUtil pUtil;
	Point robotPos;

	Segment GetNextEdge(EdgeIterator& ei, Polygon& polygon);
	double GetSlope(Segment& edge);


	double length(Segment edge);
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
