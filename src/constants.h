/**
 * constants.h
 *
 *  Created on: 12-Jan-2012
 *  Author: Apurv, Ashwani, Prateek
 */

#ifndef CONSTANTS_H_
#define CONSTANTS_H_
/**
 * File inclusions required in various algorithms
 */
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Polygon_2.h>
#include <CGAL/Polygon_with_holes_2.h>
#include <CGAL/Boolean_set_operations_2.h>
#include <CGAL/Object.h>
#include <list>
#include <CGAL/intersections.h>
#include <CGAL/squared_distance_2.h>
#include <CGAL/Polygon_2_algorithms.h>

#include <CGAL/Cartesian.h>
#include <CGAL/Aff_transformation_2.h>
#include <CGAL/Vector_2.h>

#include <CGAL/Arr_segment_traits_2.h>
#include <CGAL/Arrangement_2.h>
#include <CGAL/Arrangement_on_surface_2.h>
#include <CGAL/intersections.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Quotient.h>
#include <CGAL/MP_Float.h>

#include <boost/graph/graph_traits.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>
#include <boost/graph/connected_components.hpp>

#include <iostream>
#include <fstream>
#include <string>
#include <GL/glui.h>
#include <set>

using namespace boost;


typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
typedef K::Point_2 Point;
typedef CGAL::Polygon_2<K> Polygon;
typedef CGAL::Polygon_with_holes_2<K> Polygon_with_holes;
typedef CGAL::Polygon_2<K>::Vertex_iterator VertexIterator;
typedef CGAL::Polygon_2<K>::Edge_const_iterator EdgeIterator;
typedef CGAL::Ray_2<K> Ray;
typedef CGAL::Object Object;
typedef CGAL::Segment_2<K> Segment;

typedef CGAL::Aff_transformation_2<K>  Transformation;
typedef CGAL::Vector_2<K>              Vector;

/**
 * typedef for arrangements
 */
typedef CGAL::Quotient<CGAL::MP_Float> Number_type;
typedef CGAL::Cartesian<Number_type> Kernel;
typedef CGAL::Arr_segment_traits_2<Kernel> Traits_2;
typedef Traits_2::Point_2 Point_2;
typedef Traits_2::X_monotone_curve_2 Segment_2;
typedef CGAL::Arrangement_2<Traits_2> Arrangement;


/**
 * typedefs for using boost graph library
 */
typedef adjacency_list < listS, vecS, undirectedS,
    no_property, property < edge_weight_t, float > > graph_t;
typedef graph_traits < graph_t >::vertex_descriptor vertex_descriptor;
typedef graph_traits < graph_t >::edge_descriptor edge_descriptor;
typedef std::pair<int, int> Edge;

#endif /* CONSTANTS_H_ */
