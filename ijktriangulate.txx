/// \file ijktriangulate.txx
/// ijk templates for triangulating polyhedra
/// Version 0.2.0

/*
  IJK: Isosurface Jeneration Kode
  Copyright (C) 2010-2017 Rephael Wenger

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public License
  (LGPL) as published by the Free Software Foundation; either
  version 2.1 of the License, or any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
*/

#ifndef _IJKTRIANGULATE_
#define _IJKTRIANGULATE_

#include "ijkmesh.txx"

#include <algorithm>
#include <numeric>
#include <tuple>
#include <vector>


namespace IJK {

  // **************************************************
  // TRIANGULATE POLYGONS
  // **************************************************

  /// Triangulate a polygon by adding diagonals from vertex poly_vert[0].
  /// - Polygon vertices are listed in clockwise or counter-clockwise order
  ///   around the polygon.
  /// - poly_vert[0] is a vertex of all new triangles.
  /// - Add new triangles to vector tri_vert.
  template <typename NTYPE, typename VTYPE0, typename VTYPE1>
  void triangulate_polygon
  (const NTYPE num_poly_vert, const VTYPE0 * poly_vert, 
   std::vector<VTYPE1> & tri_vert)
  {
    VTYPE0 v0 = poly_vert[0];
    for (NTYPE i = 1; i+1 < num_poly_vert; i++) {
      add_triangle_vertices(v0, poly_vert[i], poly_vert[i+1], tri_vert);
    }
  }

  /// Triangulate a polygon using diagonals from vertex poly_vert[index_v0].
  /// - Polygon vertices are listed in clockwise or counter-clockwise order
  ///   around the polygon.
  /// - poly_vert[index_v0] is a vertex of all new triangles.
  /// - Add new triangles to vector tri_vert.
  template <typename NTYPE, 
            typename VTYPE0, typename VTYPE1, typename VTYPE2>
  void triangulate_polygon
  (const NTYPE num_poly_vert, const VTYPE0 * poly_vert,
   const VTYPE1 index_v0,
   std::vector<VTYPE2> & tri_vert)
  {
    VTYPE0 v0 = poly_vert[index_v0];
    NTYPE i1 = (index_v0+1)%num_poly_vert;
    NTYPE i2 = (i1+1)%num_poly_vert;
    while (i2 != index_v0) {
      add_triangle_vertices(v0, poly_vert[i1], poly_vert[i2], tri_vert);
      i1 = i2;
      i2 = (i1+1)%num_poly_vert;
    }
  }

  /// Triangulate a polygon by adding a triangle between each polygon edge
  ///   and vertex w0.
  /// - Polygon vertices are listed in clockwise or counter-clockwise order
  ///   around the polygon.
  /// - Add new triangles to vector tri_vert.
  template <typename NTYPE, typename VTYPE0, typename VTYPE1, 
            typename VTYPE2>
  void triangulate_polygon_with_vertex
  (const NTYPE num_poly_vert, const VTYPE0 * poly_vert, const VTYPE1 w0,
   std::vector<VTYPE2> & tri_vert)
  {
    for (NTYPE i0 = 0; i0 < num_poly_vert; i0++) {
      NTYPE i1 = (i0+1)%num_poly_vert;
      add_triangle_vertices(w0, poly_vert[i0], poly_vert[i1], tri_vert);
    }
  }

  /// Triangulate list of polygons.
  template <typename NTYPE0, typename NTYPE1, 
            typename VTYPE0, typename VTYPE1,
            typename ITYPE>
  void triangulate_polygon_list
  (const NTYPE0 * num_poly_vert, const VTYPE0 * poly_vert,
   const ITYPE * first_poly_vert, const NTYPE1 num_poly,
   std::vector<VTYPE1> & tri_vert)
  {
    for (NTYPE1 ipoly = 0; ipoly < num_poly; ipoly++) {
      triangulate_polygon
        (num_poly_vert[ipoly], poly_vert+first_poly_vert[ipoly], tri_vert);
    }
  }

  /// Triangulate list of polygons.
  /// - C++ STL vector format for num_poly_vert[], poly_vert[], 
  ///   first_poly_vert[].
  template <typename NTYPE, typename VTYPE0, typename VTYPE1,
            typename ITYPE>
  void triangulate_polygon_list
  (const std::vector<NTYPE> & num_poly_vert, 
   const std::vector<VTYPE0> & poly_vert,
   const std::vector<ITYPE> & first_poly_vert,
   std::vector<VTYPE1> & tri_vert)
  {
    triangulate_polygon_list
      (IJK::vector2pointer(num_poly_vert), IJK::vector2pointer(poly_vert),
       IJK::vector2pointer(first_poly_vert), num_poly_vert.size(), tri_vert);
  }

  /// Triangulate a set of quadrilaterals.
  /// - Quadrilateral vertices are listed in clockwise or counter-clockwise 
  ///   order around the polygon.
  /// - Add new triangles to vector tri_vert.
  template <typename NTYPE, typename VTYPE0, typename VTYPE1>
  void triangulate_quad
  (const VTYPE0 * quad_vert, const NTYPE num_quad,
   std::vector<VTYPE1> & tri_vert)
  {
    const NTYPE NUM_VERT_PER_QUAD = 4;

    for (NTYPE iquad = 0; iquad < num_quad; iquad++) {

      NTYPE k = iquad*NUM_VERT_PER_QUAD;
      triangulate_polygon(NUM_VERT_PER_QUAD, quad_vert+k, tri_vert);
    }
  }

  /// Triangulate a set of quadrilaterals.
  /// - C++ STL vector format for quad_vert.
  template <typename VTYPE0, typename VTYPE1>
  void triangulate_quad
  (const std::vector<VTYPE0> quad_vert, std::vector<VTYPE1> & tri_vert)
  {
    typedef typename std::vector<VTYPE0>::size_type SIZE_TYPE;

    const SIZE_TYPE NUM_VERT_PER_QUAD = 4;

    SIZE_TYPE num_quad = quad_vert.size()/NUM_VERT_PER_QUAD;
    triangulate_quad(IJK::vector2pointer(quad_vert), num_quad, tri_vert);
  }

  /// Triangulate a set of quadrilaterals by adding a triangle between
  ///   each quad edge and a new vertex for each quad.
  /// - Quadrilateral vertices are listed in clockwise or counter-clockwise
  ///   order around the polygon.
  /// - Quad i is triangulated using vertex (first_vertex+i).
  /// - Add new triangles to vector tri_vert.
  template <typename NTYPE, typename VTYPE0, typename VTYPE1, 
            typename VTYPE2>
  void triangulate_quad_with_vertices
  (const VTYPE0 * quad_vert, const NTYPE num_quad, 
   const VTYPE1 first_vertex, std::vector<VTYPE2> & tri_vert)
  {
    const NTYPE NUM_VERT_PER_QUAD = 4;

    for (NTYPE iquad = 0; iquad < num_quad; iquad++) {

      NTYPE k = iquad*NUM_VERT_PER_QUAD;
      triangulate_polygon_with_vertex
        (NUM_VERT_PER_QUAD, quad_vert+k, first_vertex+iquad, tri_vert);
    }
  }

  /// Triangulate a set of quadrilaterals by adding a triangle between
  ///   each quad edge and a new vertex for each quad.
  /// - C++ STL vector format for quad_vert.
  template <typename VTYPE0, typename VTYPE1, typename VTYPE2>
  void triangulate_quad_with_vertices
  (const std::vector<VTYPE0> & quad_vert,
   const VTYPE1 first_vertex, std::vector<VTYPE2> & tri_vert)
  {
    typedef typename std::vector<VTYPE0>::size_type SIZE_TYPE;

    const SIZE_TYPE NUM_VERT_PER_QUAD = 4;

    SIZE_TYPE num_quad = quad_vert.size()/NUM_VERT_PER_QUAD;
    triangulate_quad_with_vertices
      (IJK::vector2pointer(quad_vert), num_quad, first_vertex, tri_vert);
  }

  /// Triangulate a pentagon by adding triangles
  ///   (v0,v1,v2), (v0, v2, v3) and (v0, v3, v4).
  /// - Pentagon vertices are listed in clockwise or counter-clockwise 
  ///   order around the pentagon.
  /// - Add new triangles to vector tri_vert.
  template <typename VTYPE0, typename VTYPE1, typename VTYPE2,
            typename VTYPE3, typename VTYPE4, typename VTYPEB>
  void triangulate_pentagon
  (const VTYPE0 v0, const VTYPE1 v1, const VTYPE2 v2,
   const VTYPE3 v3, const VTYPE4 v4,
   std::vector<VTYPEB> & tri_vert)
  {
    add_triangle_vertices(v0, v1, v2, tri_vert);
    add_triangle_vertices(v0, v2, v3, tri_vert);
    add_triangle_vertices(v0, v3, v4, tri_vert);
  }

  /// Triangulate a pentagon by adding triangles incident on pentagon_vert[ivX].
  /// - Pentagon vertices are listed in clockwise or counter-clockwise 
  ///   order around the pentagon.
  /// - Add new triangles to vector tri_vert.
  template <typename VTYPE, typename ITYPE, typename VTYPEB>
  void triangulate_pentagon
  (const VTYPE pentagon_vert[], const ITYPE ivX,
   std::vector<VTYPEB> & tri_vert)
  {
    const ITYPE NUM_VERT_PER_PENTAGON(5);
    const ITYPE i1 = (ivX+1)%NUM_VERT_PER_PENTAGON;
    const ITYPE i2 = (ivX+2)%NUM_VERT_PER_PENTAGON;
    const ITYPE i3 = (ivX+3)%NUM_VERT_PER_PENTAGON;
    const ITYPE i4 = (ivX+4)%NUM_VERT_PER_PENTAGON;

    triangulate_pentagon
      (pentagon_vert[ivX], pentagon_vert[i1], pentagon_vert[i2],
       pentagon_vert[i3], pentagon_vert[i4], tri_vert);
  }

  /// Triangulate a pentagon.
  /// - Triangles are specified by ears.
  /// @param ear0 Index of first ear in triangulation. In range [0..5].
  /// @param ear1 Index of second ear in triangulation. In range [0..5].
  template <typename VTYPE0, typename VTYPE1, typename VTYPE2,
            typename VTYPE3, typename VTYPE4, 
            typename EAR_TYPE0, typename EAR_TYPE1,
            typename VTYPEB>
  void triangulate_pentagon_by_ears
  (const VTYPE0 v0, const VTYPE1 v1, const VTYPE2 v2,
   const VTYPE3 v3, const VTYPE4 v4,
   const EAR_TYPE0 ear0, const EAR_TYPE1 ear1,
   std::vector<VTYPEB> & tri_vert)
  {
    switch(ear0) {

    case 0:
      if (ear1 == 1 || ear1 == 3) 
        { triangulate_pentagon(v4, v0, v1, v2, v3, tri_vert); }
      else
        { triangulate_pentagon(v1, v2, v3, v4, v0, tri_vert); }
      break;

    case 1:
      if (ear1 == 2 || ear1 == 4) 
        { triangulate_pentagon(v0, v1, v2, v3, v4, tri_vert); }
      else
        { triangulate_pentagon(v2, v3, v4, v0, v1, tri_vert); }
      break;

    case 2:
      if (ear1 == 0 || ear1 == 3) 
        { triangulate_pentagon(v1, v2, v3, v4, v0, tri_vert); }
      else
        { triangulate_pentagon(v3, v4, v0, v1, v2, tri_vert); }
      break;

    case 3:
      if (ear1 == 0 || ear1 == 2) 
        { triangulate_pentagon(v4, v0, v1, v2, v3, tri_vert); }
      else
        { triangulate_pentagon(v2, v3, v4, v0, v1, tri_vert); }
      break;

    case 4:
    default:
      if (ear1 == 0 || ear1 == 2) 
        { triangulate_pentagon(v3, v4, v0, v1, v2, tri_vert); }
      else
        { triangulate_pentagon(v0, v1, v2, v3, v4, tri_vert); }
      break;
    }
  }

  /// Triangulate a hexagon by adding triangle (iv0,iv2,iv4).
  /// - Hexagon vertices are listed in clockwise or counter-clockwise 
  ///   order around the hexagon.
  /// - Add new triangles to vector tri_vert.
  template <typename VTYPE0, typename VTYPE1, typename VTYPE2,
            typename VTYPE3, typename VTYPE4, typename VTYPE5,
            typename VTYPEB>
  void triangulate_hexagon_using_triangle_024
  (const VTYPE0 v0, const VTYPE1 v1, const VTYPE2 v2,
   const VTYPE3 v3, const VTYPE4 v4, const VTYPE5 v5,
   std::vector<VTYPEB> & tri_vert)
  {
    add_triangle_vertices(v0, v1, v2, tri_vert);
    add_triangle_vertices(v2, v3, v4, tri_vert);
    add_triangle_vertices(v4, v5, v0, tri_vert);
    add_triangle_vertices(v0, v2, v4, tri_vert);
  }

  /// Triangulate a hexagon by adding triangle (iv4,iv2,iv0).
  /// - Triangles have reverse orientation of hexagon.
  /// - Hexagon vertices are listed in clockwise or counter-clockwise 
  ///   order around the hexagon.
  /// - Add new triangles to vector tri_vert.
  template <typename VTYPE0, typename VTYPE1, typename VTYPE2,
            typename VTYPE3, typename VTYPE4, typename VTYPE5,
            typename VTYPEB>
  void triangulate_hexagon_using_triangle_024_reverse_orient
  (const VTYPE0 v0, const VTYPE1 v1, const VTYPE2 v2,
   const VTYPE3 v3, const VTYPE4 v4, const VTYPE5 v5,
   std::vector<VTYPEB> & tri_vert)
  {
    add_triangle_vertices(v2, v1, v0, tri_vert);
    add_triangle_vertices(v4, v3, v2, tri_vert);
    add_triangle_vertices(v0, v5, v4, tri_vert);
    add_triangle_vertices(v4, v2, v0, tri_vert);
  }

  /// Triangulate a hexagon by adding triangle (iv0,iv2,iv4).
  /// - Version with flag_reverse_orient.
  /// @param flag_reverse_orient 
  ///   If true, reverse orientation in creating triangles from the hexagon.
  template <typename VTYPE0, typename VTYPE1, typename VTYPE2,
            typename VTYPE3, typename VTYPE4, typename VTYPE5,
            typename VTYPEB>
  void triangulate_hexagon_using_triangle_024
  (const VTYPE0 v0, const VTYPE1 v1, const VTYPE2 v2,
   const VTYPE3 v3, const VTYPE4 v4, const VTYPE5 v5,
   const bool flag_reverse_orient,
   std::vector<VTYPEB> & tri_vert)
  {
    if (flag_reverse_orient) {
      triangulate_hexagon_using_triangle_024_reverse_orient
        (v0, v1, v2, v3, v4, v5, tri_vert);
    }
    else {
      triangulate_hexagon_using_triangle_024
        (v0, v1, v2, v3, v4, v5, tri_vert);
    }
  }

  /// Triangulate a hexagon.
  /// - Triangles are specified by ears.
  /// - Hexagon vertices are listed in clockwise or counter-clockwise 
  ///   order around the hexagon.
  /// - Add new triangles to vector tri_vert.
  /// @param ear0 Index of first ear in triangulation. In range [0..5].
  /// @param ear1 Index of second ear in triangulation. In range [0..5].
  /// @param ear2 Index of third ear in triangulation. In range [0..5].
  template <typename VTYPE0, typename VTYPE1, typename VTYPE2,
            typename VTYPE3, typename VTYPE4, typename VTYPE5,
            typename EAR_TYPE0, typename EAR_TYPE1, typename EAR_TYPE2,
            typename VTYPEB>
  void triangulate_hexagon_by_ears
  (const VTYPE0 v0, const VTYPE1 v1, const VTYPE2 v2,
   const VTYPE3 v3, const VTYPE4 v4, const VTYPE5 v5,
   const EAR_TYPE0 ear0, const EAR_TYPE1 ear1, const EAR_TYPE2 ear2,
   std::vector<VTYPEB> & tri_vert)
  {
    EAR_TYPE1 ear1B;
    EAR_TYPE2 ear2B;

    if (ear1 > ear0) { ear1B = ear1-1; }
    else { ear1B = ear1; }
    if (ear2 > ear0) { ear2B = ear2-1; }
    else { ear2B = ear2; }

    switch(ear0) {

    case 0:
      add_triangle_vertices(v5, v0, v1, tri_vert);
      triangulate_pentagon_by_ears
        (v1, v2, v3, v4, v5, ear1B, ear2B, tri_vert);
      break;

    case 1:
      add_triangle_vertices(v0, v1, v2, tri_vert);
      triangulate_pentagon_by_ears
        (v0, v2, v3, v4, v5, ear1B, ear2B, tri_vert);
      break;

    case 2:
      add_triangle_vertices(v1, v2, v3, tri_vert);
      triangulate_pentagon_by_ears
        (v0, v1, v3, v4, v5, ear1B, ear2B, tri_vert);
      break;

    case 3:
      add_triangle_vertices(v2, v3, v4, tri_vert);
      triangulate_pentagon_by_ears
        (v0, v1, v2, v4, v5, ear1B, ear2B, tri_vert);
      break;

    case 4:
      add_triangle_vertices(v3, v4, v5, tri_vert);
      triangulate_pentagon_by_ears
        (v0, v1, v2, v3, v5, ear1B, ear2B, tri_vert);
      break;

    case 5:
    default:
      add_triangle_vertices(v4, v5, v0, tri_vert);
      triangulate_pentagon_by_ears
        (v0, v1, v2, v3, v4, ear1B, ear2B, tri_vert);
      break;
    }
  }

  /// Triangulate a hexagon.
  /// - Version with array of hexagon vertices.
  /// - Hexagon vertices are listed in clockwise or counter-clockwise 
  ///   order around the hexagon.
  template <typename VTYPE,
            typename EAR_TYPE0, typename EAR_TYPE1, typename EAR_TYPE2,
            typename VTYPEB>
  void triangulate_hexagon_by_ears
  (const VTYPE hex_vert[],
   const EAR_TYPE0 ear0, const EAR_TYPE1 ear1, const EAR_TYPE2 ear2,
   std::vector<VTYPEB> & tri_vert)
  {
    triangulate_hexagon_by_ears
      (hex_vert[0], hex_vert[1], hex_vert[2], hex_vert[3], hex_vert[4],
       hex_vert[5], ear0, ear1, ear2, tri_vert);
  }

}

#endif
