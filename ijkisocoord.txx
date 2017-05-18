/// \file ijkisocoord.txx
/// ijk templates for computing isosurface coordinates
/// Version 0.2.0

/*
  IJK: Isosurface Jeneration Kode
  Copyright (C) 2015-2016 Rephael Wenger

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

#ifndef _IJKISOCOORD_
#define _IJKISOCOORD_

#include <vector>

#include "ijk.txx"
#include "ijkcoord.txx"
#include "ijkinterpolate.txx"

namespace IJK {

  // **************************************************************
  /// @name SUBROUTINES FOR COMPUTING ISOSURFACE VERTEX COORDINATES
  // **************************************************************

  //! Compute coordinates of isosurface vertices on line segment (iv0, iv1)
  //!   using linear interpolation.
  //! @param scalar_grid Scalar grid data.
  //! @param ivA First grid vertex.
  //! @param ivB Second grid vertex
  //! @param[out] coord[] Array of isosurface vertex coordinates.
  //!  coord[d] = d'th coordinate of vertex i'th vertex.
  //! @pre Array coord[] is preallocated to length dimension.
  template <typename SGRID_TYPE, typename ISOVALUE_TYPE,
            typename VTYPE, typename CTYPE>
  void compute_isov_coord_on_line_segment_linear
  (const SGRID_TYPE & scalar_grid, const ISOVALUE_TYPE isovalue,
   const VTYPE iv0, const VTYPE iv1, CTYPE * coord)
  {
    typedef typename SGRID_TYPE::DIMENSION_TYPE DTYPE;
    typedef typename SGRID_TYPE::SCALAR_TYPE STYPE;

    const DTYPE dimension = scalar_grid.Dimension();
    const STYPE * scalar = scalar_grid.ScalarPtrConst();
    IJK::ARRAY<VTYPE> coord0(dimension);
    IJK::ARRAY<VTYPE> coord1(dimension);
  
    STYPE s0 = scalar[iv0];
    STYPE s1 = scalar[iv1];

    scalar_grid.ComputeCoord(iv0, coord0.Ptr());
    scalar_grid.ComputeCoord(iv1, coord1.Ptr());

    linear_interpolate_coord
      (dimension, s0, coord0.PtrConst(), s1, coord1.PtrConst(), 
       isovalue, coord);
  }

  //! Compute coordinates of isosurface vertices on list of line segments
  //!   using linear interpolation.
  //! @param scalar_grid Scalar grid data.
  //! @param endpoint[] Array of line segment endpoints.
  //!   Line segment i has endpoints (2*endpoint[i], 2*endpoint[i]+1).
  //! @param[out] coord[] Array of isosurface vertex coordinates.
  //!  coord[i*dimension+j] = j'th coordinate of vertex i'th vertex
  //! @pre Array coord[] is preallocated to length at least
  //!    dimension*(endpoint.size()/2).
  template <typename SGRID_TYPE, typename ISOVALUE_TYPE,
            typename VTYPE, typename CTYPE>
  void compute_isov_coord_on_line_segment_linear
  (const SGRID_TYPE & scalar_grid, const ISOVALUE_TYPE isovalue,
   const std::vector<VTYPE> & endpoint, CTYPE * coord)
  {
    typedef typename SGRID_TYPE::DIMENSION_TYPE DTYPE;
    typedef typename SGRID_TYPE::NUMBER_TYPE NTYPE;

    const DTYPE dimension = scalar_grid.Dimension();
    const NTYPE num_line_segments = endpoint.size()/2;
  
    for (NTYPE i = 0; i < num_line_segments; i++) {
      compute_isov_coord_on_line_segment_linear
        (scalar_grid, isovalue, endpoint[2*i], endpoint[2*i+1], 
         coord+dimension*i);
    }
  }

  /// Compute coordinates of isosurface vertices on a single grid edge
  ///   using linear interpolation.
  /// @param scalar_grid Scalar grid data.
  /// @param grid_edge Grid edge.
  /// @param[out] coord[] Array of isosurface vertex coordinates.
  ///  coord[d] = d'th coordinate of isosurface vertex.
  /// @pre Array coord[] is preallocated to length at least dimension.
  template <typename SGRID_TYPE, typename ISOVALUE_TYPE,
            typename GRID_EDGE_TYPE, typename CTYPE>
  void compute_isov_coord_on_grid_edge_linear
  (const SGRID_TYPE & scalar_grid, const ISOVALUE_TYPE isovalue,
   const GRID_EDGE_TYPE & grid_edge, CTYPE * coord)
  {
    typedef typename SGRID_TYPE::DIMENSION_TYPE DTYPE;
    typedef typename SGRID_TYPE::VERTEX_INDEX_TYPE VTYPE;
  
    VTYPE iv0 = grid_edge.Endpoint0();
    DTYPE edge_direction = grid_edge.Direction();
    VTYPE iv1 = scalar_grid.NextVertex(iv0, edge_direction);

    compute_isov_coord_on_line_segment_linear
      (scalar_grid, isovalue, iv0, iv1, coord);
  }

  /// Compute coordinates of isosurface vertices on list of edges
  ///   using linear interpolation.
  /// @param scalar_grid Scalar grid data.
  /// @param grid_edge[] List of grid edges.
  ///    Edge i has lower/leftmost endpoint grid_edge[i].Endpoint0()
  ///    and direction grid_edge[i].Direction().
  /// @param[out] coord[] Array of isosurface vertex coordinates.
  ///  coord[i*dimension+j] = j'th coordinate of vertex i'th vertex
  /// @pre Array coord[] is preallocated to length at least
  ///    dimension*(grid_edge.size()/2).
  template <typename SGRID_TYPE, typename ISOVALUE_TYPE,
            typename GRID_EDGE_TYPE, typename CTYPE>
  void compute_isov_coord_on_grid_edge_linear
  (const SGRID_TYPE & scalar_grid, const ISOVALUE_TYPE isovalue,
   const std::vector<GRID_EDGE_TYPE> & grid_edge, CTYPE * coord)
  {
    typedef typename SGRID_TYPE::DIMENSION_TYPE DTYPE;
    typedef typename SGRID_TYPE::NUMBER_TYPE NTYPE;

    const DTYPE dimension = scalar_grid.Dimension();
    const NTYPE nume = grid_edge.size();

    for (NTYPE i = 0; i < nume; i++) {
      compute_isov_coord_on_grid_edge_linear
        (scalar_grid, isovalue, grid_edge[i], coord+dimension*i);
    }
  }

}

#endif

