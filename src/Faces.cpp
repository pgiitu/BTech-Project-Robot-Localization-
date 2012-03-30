/*
 * Faces.cpp
 *
 *  Created on: 22-Jan-2012
 *      Author: prateek
 */

#include "Faces.h"
using std::cout;
/**
 * Default Constructor
 */
Faces::Faces() {
	// TODO Auto-generated constructor stub

}
/**
 * Constructor which accepts a polygon
 */
Faces::Faces(Polygon p)
{
	face=p;
}
/**
 * @params face: It represnts polygon corresponding to the face
 * @params noOfHypothesis: It shows the number of hypothesis
 * @containedIn: A bool array showing the response of hypothesis to this face
 * @partOfMajorityMap: Boolean showing the part of majority map
 */
Faces::Faces(int n, Polygon p, bool *A,bool partMmap)
{
	containedIn=new bool[n];
	noOfHypothesis=n;
	face=p;
	for(int i=0;i<noOfHypothesis;i++)
	{
		containedIn[i]=*(A+i);
	}
	partOfMajorityMap=partMmap;
}
/**
 * Destructor
 */
Faces::~Faces() {
}
/**
 * A method to print the description of face
 */
void Faces::PrintDescription()
{
	std::cout<<"The face is part of majority map:"<<partOfMajorityMap<<"\n";
	for(int i=0;i<noOfHypothesis;i++)
	{
		cout<<containedIn[i]<<"  ";
	}
	cout<<"\n";
}
