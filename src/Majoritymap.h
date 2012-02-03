/*
 * Majoritymap.h
 *
 *  Created on: 22-Jan-2012
 *      Author: prateek
 */

#ifndef MAJORITYMAP_H_
#define MAJORITYMAP_H_
#include "constants.h"
#include "Faces.h"
#include "HypothesisGenerator.h"
#include <list>

typedef CGAL::Quotient<CGAL::MP_Float>                  Number_type;

using namespace std;
//using namespace boost::numeric::ublas;

class Majoritymap {
public:

	list<Faces> listMmapFaces;
	Polygon map;
	int noOfHypothesis;
	Point *hypothesis;
	Point center;
	list<Polygon> listTanslatedPolygons;
	Arrangement mmapArrangement;

	//HypothesisGenerator hGenerator;

	Majoritymap();
	Majoritymap(int n, Point H[],Point c,Polygon P);
	void PrintMajorityMap();
	void GenerateMajorityMap();
	Polygon GetTranslatePolygon(Transformation& translate, Polygon& polygon);
//	bool ContainmentTest();
	Polygon ConvertFaceToPolygon(Arrangement::Ccb_halfedge_const_circulator circ);
	bool IsContainedIn(Polygon outer,Polygon inner);
	bool CheckPartOfMajorityMap(int agree, int noOfHypothesis);

//	void print_face (Arrangement::Face_const_handle f);
//	void print_ccb (Arrangement::Ccb_halfedge_const_circulator circ);
	virtual ~Majoritymap();
};

#endif /* MAJORITYMAP_H_ */
