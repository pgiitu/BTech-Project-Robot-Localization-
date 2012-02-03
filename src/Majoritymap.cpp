/*
 * Majoritymap.cpp
 *
 *  Created on: 22-Jan-2012
 *      Author: prateek
 */

#include "Majoritymap.h"

Majoritymap::Majoritymap() {
	// TODO Auto-generated constructor stub

}

Majoritymap::Majoritymap(int n, Point H[],Point c,Polygon P)
{
	noOfHypothesis=n;
	center=c;
	map=P;
	hypothesis=H;
}

Majoritymap::~Majoritymap() {
	// TODO Auto-generated destructor stub
}

void Majoritymap::PrintMajorityMap()
{
	list<Faces>::iterator f;
	int i;
	for(f=listMmapFaces.begin(),i=1;f!=listMmapFaces.end();++f,++i)
	{
		std::cout<<"Face No  "<<i<<"\n";
		f->PrintDescription();
	}
}

void Majoritymap::GenerateMajorityMap()
{
	/*
	 * Generating the translated polygons according to the center
	 */
//	TranslatePolygons(Point T[],Polygon )

	for(int i=0;i<noOfHypothesis;i++)
	{
		Vector T((hypothesis[i].cartesian(0)-center.cartesian(0)),(hypothesis[i].cartesian(1)-center.cartesian(1)));
		Transformation translate(CGAL::TRANSLATION, T);
		Polygon p=GetTranslatePolygon(translate, map);
		listTanslatedPolygons.push_back(p);
	}

	list<Polygon>::iterator pi;
	for(pi=listTanslatedPolygons.begin();pi!=listTanslatedPolygons.end();++pi)
	{
		for (EdgeIterator ei = pi->edges_begin(); ei != pi->edges_end(); ++ei)
		{
			//Segment_2 s(ei->start(),ei->end());
			Point s=ei->start();
			Point d=ei->end();
			Point_2 source(s.cartesian(0),s.cartesian(1));
			Point_2 destination(d.cartesian(0),d.cartesian(1));
			Segment_2 seg(source, destination);
			CGAL::insert (mmapArrangement,seg);
		}
		/*		CGAL::insert(mmapArrangement,pi->edges_begin(),pi->edges_end());*/
	}

	/*
	 * now mmapArrangement contains all the faces required by us
	 */

	Arrangement::Face_const_iterator fit;
	std::cout << mmapArrangement.number_of_faces() << " faces:" << std::endl;
	for (fit = mmapArrangement.faces_begin(); fit != mmapArrangement.faces_end(); ++fit)
	{
		  if (!fit->is_unbounded())
		  {
			Polygon p=ConvertFaceToPolygon(fit->outer_ccb());
			bool liesIn[noOfHypothesis];

			list<Polygon>::iterator pi;
			int i=0;
			int agreedByHypothesis=0;
			for(pi=listTanslatedPolygons.begin();pi!=listTanslatedPolygons.end();++pi)
			{
				if(IsContainedIn((*pi),p))  //return true if p lies in pi
				{
					liesIn[i++]=true;
					agreedByHypothesis++;
				}
				else
				{
					liesIn[i++]=false;
				}
			}
			cout<<"lies in array is  ";
			for(i=0;i<noOfHypothesis;i++)
			{
				cout<<liesIn[i]<<"   ";
			}
			cout<<endl;
			bool isPartMmap=CheckPartOfMajorityMap(agreedByHypothesis,noOfHypothesis);
	//		cout<<"Is part of "<<isPartMmap<<endl;
			Faces f(noOfHypothesis,p,liesIn,isPartMmap);
			listMmapFaces.push_back(f); //constructing the majority map by adding the faces as polygons to it
		  }
		  else
		  {
			  //   std::cout << "Unbounded face. " << std::endl;

		  }

		}
	}

bool Majoritymap::CheckPartOfMajorityMap(int agree, int noOfHypothesis)
{
	if(noOfHypothesis%2==0)
	{
		if(agree>= noOfHypothesis/2)
			return true;
	}
	else
	{
		if(agree> noOfHypothesis/2)
			return true;
	}
	return false;
}

/*
 * This function when given a face converts it into a polygon and returns the polygon
 */
Polygon Majoritymap::ConvertFaceToPolygon(Arrangement::Ccb_halfedge_const_circulator circ)
{
	  Polygon poly;
	  Arrangement::Ccb_halfedge_const_circulator curr = circ;
	  do {
		  Point_2 p=(curr->source()->point());
		  Number_type x=p.cartesian(0);
		  Number_type y=p.cartesian(1);
		  poly.push_back(Point(to_double(x),to_double(y)));
	  } while (++curr != circ);
	  return poly;
}

/*
 * this function checks whether inner polygon is contained in outer polygon.
 * Note:- It only checks for the vertices of the inner polygon to be contained in the outer polygon as we don't have
 * the case when an edge will lie outside the outer polygon when the end points are in the polygon
 */
bool Majoritymap::IsContainedIn(Polygon outer,Polygon inner)
{
	for(VertexIterator vi = inner.vertices_begin(); vi !=inner.vertices_end(); ++vi)
	{
		if(outer.has_on_unbounded_side(*vi))
		{
			return false;
		}
	}
	return true;
}

/*
 * This function applies the transformation on a polygon and returns a new polygon
 */
Polygon Majoritymap::GetTranslatePolygon(Transformation& translate, Polygon& polygon){

	Polygon translatedPolygon;

	for(VertexIterator vi = polygon.vertices_begin(); vi !=polygon.vertices_end(); ++vi){
		translatedPolygon.push_back(translate(*vi));
	}
	return translatedPolygon;
}


/*
void Majoritymap::print_face (Arrangement::Face_const_handle f)
{
  // Print the outer boundary.
  if (f->is_unbounded())
  {
    std::cout << "Unbounded face. " << std::endl;
  }
  else
  {
    std::cout << "Outer boundary: "<<std::endl;
    print_ccb (f->outer_ccb());
  }

  // Print the boundary of each of the holes.
  Arrangement::Hole_const_iterator hi;
  int index = 1;
  for (hi = f->holes_begin(); hi != f->holes_end(); ++hi, ++index)
  {
    std::cout << "    Hole #" << index << ":";
    print_ccb (*hi);
  }

  // Print the isolated vertices.
  Arrangement::Isolated_vertex_const_iterator iv;
  for (iv = f->isolated_vertices_begin(), index = 1;iv != f->isolated_vertices_end(); ++iv, ++index)
  {
    std::cout << "    Isolated vertex #" << index << ": "
              << "(" << iv->point() << ")" << std::endl;
  }

}

void Majoritymap::print_ccb (Arrangement::Ccb_halfedge_const_circulator circ)
{
  Arrangement::Ccb_halfedge_const_circulator curr = circ;
  do {
//	  std::cout << "(" << curr->source()->point() << ")";
	  std::cout << "(" << curr->source().cartesian(0) << ")";
//	  Arrangement_2::Halfedge_const_handle he = curr->handle();
//	  std::cout << "   [" << he->curve() << "]   "
//               	<< "(" << he->target()->point() << ")";
  } while (++curr != circ);
  std::cout << std::endl;
}

*/

