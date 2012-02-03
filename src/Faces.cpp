/*
 * Faces.cpp
 *
 *  Created on: 22-Jan-2012
 *      Author: prateek
 */

#include "Faces.h"
using std::cout;
Faces::Faces() {
	// TODO Auto-generated constructor stub

}

Faces::Faces(int n, Polygon p, bool *A,bool partMmap)
{
	containedIn=new bool[n];
	noOfHypothesis=n;
	face=p;
	for(int i=0;i<noOfHypothesis;i++)
	{
		containedIn[i]=*(A+i);
		//cout<<"A[i] is  "<<A[i]<<" contained in "<<containedIn[i];
	}
	partOfMajorityMap=partMmap;
}

Faces::~Faces() {
	// TODO Auto-generated destructor stub
	std::cout << "Destructor being called \n\n\n\n\n\n";
//	delete [] containedIn;
//	containedIn=0;
}

void Faces::PrintDescription()
{
	std::cout<<"The face is part of majority map:"<<partOfMajorityMap<<"\n";
	for(int i=0;i<noOfHypothesis;i++)
	{
		cout<<containedIn[i]<<"  ";
	}
	cout<<"\n";
}
