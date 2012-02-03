/*
 * Faces.h
 *
 *  Created on: 22-Jan-2012
 *      Author: prateek
 */

#ifndef FACES_H_
#define FACES_H_
#include "constants.h"

class Faces {
public:
	Polygon face;
	int noOfHypothesis;
	bool *containedIn;
	bool partOfMajorityMap;
	Faces();
	Faces(int n,Polygon p, bool *A,bool partMmap);
	void PrintDescription();
	virtual ~Faces();
};

#endif /* FACES_H_ */
