/**
 * Faces.h
 *
 *  Created on: 22-Jan-2012
 *  Author: Prateek, Ashwani, Apurv
 */

#ifndef FACES_H_
#define FACES_H_
#include "constants.h"

class Faces {
public:
	/**
	 * Class members
	 * @params face: It represnts polygon corresponding to the face
	 * @params noOfHypothesis: It shows the number of hypothesis
	 * @containedIn: A bool array showing the response of hypothesis to this face
	 * @partOfMajorityMap: Boolean showing the part of majority map
	 */
	Polygon face;
	int noOfHypothesis;
	bool *containedIn;
	bool partOfMajorityMap;



	Faces();
	Faces(int n,Polygon p, bool *A,bool partMmap);
	Faces(Polygon p);
	void PrintDescription();
	virtual ~Faces();
};

#endif /* FACES_H_ */
