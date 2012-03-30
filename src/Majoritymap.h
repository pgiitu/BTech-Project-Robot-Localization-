/**
 * Majoritymap.h
 *
 *  Created on: 22-Jan-2012
 *      Author: prateek,ashwani,apurv
 */

#ifndef MAJORITYMAP_H_
#define MAJORITYMAP_H_
#include "constants.h"
#include "Faces.h"
#include "HypothesisGenerator.h"
#include <list>

typedef CGAL::Quotient<CGAL::MP_Float>                  Number_type;

using namespace std;

class Majoritymap {
public:

	list<Faces> listMmapFaces;
	Polygon map;
	int noOfHypothesis;
	Point *hypothesis;
	Point center;
	list<Polygon> listTanslatedPolygons;
	Arrangement mmapArrangement;

	PolygonUtil pUtil;

	Majoritymap();
	Majoritymap(int n, Point H[],Point c,Polygon P);
	Majoritymap(int n,std::list<Polygon> PolygonList);


	void PrintMajorityMap();
	void GenerateMajorityMap();
	Polygon GetTranslatePolygon(Transformation& translate, Polygon& polygon);
	Polygon ConvertFaceToPolygon(Arrangement::Ccb_halfedge_const_circulator circ);
	bool IsContainedIn(Polygon outer,Polygon inner);
	bool CheckPartOfMajorityMap(int agree, int noOfHypothesis);

	list<Polygon> findRegionContaningOrigin();
	bool areAdjacent(Polygon& poly1, Polygon& poly2);

	Polygon OverlayContaningOrigin(Point &center);
	Polygon generateUnionFaces();
	void GenerateOverlay(list<Polygon> polygonList);
	void partMajority();
	virtual ~Majoritymap();
};

#endif /* MAJORITYMAP_H_ */
