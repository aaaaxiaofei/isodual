/// \file ijkfacet_intersection_table.txx
/// Table for determining vertices on facet intersections.
/// Version 0.1.0

/*
  IJK: Isosurface Jeneration Kode
  Copyright (C) 2016 Rephael Wenger

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

#ifndef _IJKFACET_INTERSECTION_TABLE_
#define _IJKFACET_INTERSECTION_TABLE_

#include "ijk.txx"
#include "ijkcube.txx"

namespace IJK {

  // *********************************************************************
  // TEMPLATE CLASS FACET_INTERSECTION_TABLE
  // *********************************************************************

  /// Class to report vertices in facet intersections.
  /// @tparam VBITS_TYPE Type to store vertex bits.
  template <typename DTYPE, typename VBITS_TYPE, typename NUM_TYPE>
  class FACET_INTERSECTION_TABLE {

  protected:
    DTYPE dimension;
    NUM_TYPE num_cube_vertices;
    NUM_TYPE num_table_entries;
    VBITS_TYPE * in_ridge_bits;
    VBITS_TYPE * in_facet_intersection_bits;

    /// True if array entry[] is allocated.
    bool is_table_allocated;
    
    /// Initialization routine.
    template <typename DTYPE2>
    void Init(const DTYPE2 dimension);

    void Clear();                           ///< Clear routine.
    void ComputeInRidgeBits();              ///< Compute in_ridge_bits[].

    /// Compute in_facet_intersection_bits[].
    void ComputeInFacetIntersectionBits();

  public:
    typedef NUM_TYPE NUMBER_TYPE;
    typedef VBITS_TYPE VERTEX_BITS_TYPE;

  public:
    FACET_INTERSECTION_TABLE() { Init(3); };
    template <typename DTYPE2>
    FACET_INTERSECTION_TABLE(const DTYPE2 dimension)
    { Init(dimension); }
    ~FACET_INTERSECTION_TABLE() { Clear(); }


    // get functions
    DTYPE Dimension() const { return(dimension); }
    NUM_TYPE NumCubeVertices() const { return(num_cube_vertices); }
    NUM_TYPE NumTableEntries() const { return(num_table_entries); }
    VBITS_TYPE InRidgeBits(const NUM_TYPE ientry) const
    { return(in_ridge_bits[ientry]); }
    VBITS_TYPE InFacetIntersectionBits(const NUM_TYPE ientry) const
    { return(in_facet_intersection_bits[ientry]); }
    bool IsTableAllocated() const { return(is_table_allocated); }

    template <typename DTYPE2>
    void SetDimension(const DTYPE2 dimension);

    void Create();

    // Check routines.
    template <typename DTYPE2>
    void CheckVertexBitsType(const DTYPE2 dimension) const;
    template <typename DTYPE2>
    void CheckNumType(const DTYPE2 dimension) const;
  };


  // *********************************************************************
  // TEMPLATE CLASS FACET_INTERSECTION_TABLE MEMBER FUNCTIONS
  // *********************************************************************

  template <typename DTYPE, typename VBITS_TYPE, typename NUM_TYPE>
  template  <typename DTYPE2>
  void FACET_INTERSECTION_TABLE<DTYPE,VBITS_TYPE,NUM_TYPE>::
  Init(const DTYPE2 dimension)
  {
    in_ridge_bits = NULL;
    in_facet_intersection_bits = NULL;
    is_table_allocated = false;
    num_table_entries = 0;
    SetDimension(dimension);
  }

  template <typename DTYPE, typename VBITS_TYPE, typename NUM_TYPE>
  template  <typename DTYPE2>
  void FACET_INTERSECTION_TABLE<DTYPE,VBITS_TYPE,NUM_TYPE>::
  SetDimension(const DTYPE2 dimension)
  {
    Clear();
    this->dimension = dimension;
    num_cube_vertices = IJK::compute_num_cube_vertices(dimension);
    CheckVertexBitsType(dimension);
    CheckNumType(dimension);
  }


  template <typename DTYPE, typename VBITS_TYPE, typename NUM_TYPE>
  void FACET_INTERSECTION_TABLE<DTYPE,VBITS_TYPE,NUM_TYPE>::Clear()
  {
    if (in_ridge_bits != NULL) { delete [] in_ridge_bits; }
    in_ridge_bits = NULL;
    if (in_facet_intersection_bits != NULL) 
      { delete [] in_facet_intersection_bits; }
    in_facet_intersection_bits = NULL;
    num_table_entries = 0;
    dimension = 0;
    num_cube_vertices = 1;
    is_table_allocated = false;
  }

  template <typename DTYPE, typename VBITS_TYPE, typename NUM_TYPE>
  void FACET_INTERSECTION_TABLE<DTYPE,VBITS_TYPE,NUM_TYPE>::
  ComputeInRidgeBits()
  {
    IJK::UNIT_CUBE<DTYPE,NUM_TYPE,NUM_TYPE> unit_cube(Dimension());

    for (NUM_TYPE ientry = 0; ientry < NumTableEntries(); ientry++) {
      in_ridge_bits[ientry] = 0;
      for (NUM_TYPE jv = 0; jv < NumCubeVertices(); jv++) {
        NUM_TYPE num_selected_facet_directions = 0;
        for (DTYPE orth_dir = 0; orth_dir < Dimension(); orth_dir++) {
          // Bit 2*orth_dir is 1, if lower/leftmost facet 
          //   with orth direction orth_dir is on grid boundary.
          NUM_TYPE lower_facet_mask = (NUM_TYPE(1) << (2*orth_dir));
          // Bit 2d is 1, if lower/leftmost facet 
          //   with orth direction orth_dir is on grid boundary.
          NUM_TYPE upper_facet_mask = (NUM_TYPE(1) << (2*orth_dir+1));
          NUM_TYPE c = unit_cube.VertexCoord(jv,orth_dir);
          if (c == 0) {
            if ((lower_facet_mask & ientry) != 0)
              { num_selected_facet_directions++; }
          }
          else {
            if ((upper_facet_mask & ientry) != 0)
              { num_selected_facet_directions++; }
          }
        }

        if (num_selected_facet_directions > 1) {
          // Vertex jv is on ridge.
          VBITS_TYPE vertex_mask = (VBITS_TYPE(1) << jv);
          in_ridge_bits[ientry] = (in_ridge_bits[ientry] | vertex_mask);
        }
      }
    }

  }


  template <typename DTYPE, typename VBITS_TYPE, typename NUM_TYPE>
  void FACET_INTERSECTION_TABLE<DTYPE,VBITS_TYPE,NUM_TYPE>::
  ComputeInFacetIntersectionBits()
  {
    IJK::UNIT_CUBE<DTYPE,NUM_TYPE,NUM_TYPE> unit_cube(Dimension());
    const NUM_TYPE num_cube_facets = unit_cube.NumFacets();

    for (NUM_TYPE ientry = 0; ientry < NumTableEntries(); ientry++) {
      in_facet_intersection_bits[ientry] = 0;

      NUM_TYPE num_selected_facets = 0;
      NUM_TYPE ival = ientry;
      for (NUM_TYPE jf = 0; jf < num_cube_facets; jf++) {
        if ((ival %2) == 1) { num_selected_facets++; }
        ival = (ival >> 1);
      }

      if (num_selected_facets > 0) {

        for (NUM_TYPE jv = 0; jv < NumCubeVertices(); jv++) {
          NUM_TYPE num_facets_containing_vertex = 0;
          for (NUM_TYPE jf = 0; jf < num_cube_facets; jf++) {
            NUM_TYPE facet_mask = (NUM_TYPE(1) << jf);

            DTYPE orth_dir = NUM_TYPE(jf/2);
            if ((facet_mask & ientry) != 0) {
              NUM_TYPE c = unit_cube.VertexCoord(jv,orth_dir);

              if (c == 0 && (jf%2 == 0))
                { num_facets_containing_vertex++; }
              else if (c == 1 && (jf%2 == 1))
                { num_facets_containing_vertex++; }
            }
          }

          if (num_selected_facets == num_facets_containing_vertex) {
            // Vertex jv is in all selected facets.
            VBITS_TYPE vertex_mask = (VBITS_TYPE(1) << jv);
            in_facet_intersection_bits[ientry] = 
              (in_facet_intersection_bits[ientry] | vertex_mask);
          }
        }
      }
    }

  }

  template <typename DTYPE, typename VBITS_TYPE, typename NUM_TYPE>
  void FACET_INTERSECTION_TABLE<DTYPE,VBITS_TYPE,NUM_TYPE>::Create()
  {
    IJK::PROCEDURE_ERROR error("FACET_INTERSECTION_TABLE::Create");

    if (is_table_allocated || in_ridge_bits != NULL ||
        in_facet_intersection_bits != NULL) {
      error.AddMessage
        ("Programming error.  Facet intersection table already created.");
      throw error;
    }

    CheckVertexBitsType(Dimension());
    CheckNumType(Dimension());

    NUM_TYPE num_cube_facets = compute_num_cube_facets(Dimension());

    num_table_entries = (NUM_TYPE(1) << num_cube_facets);
    in_ridge_bits = new VBITS_TYPE[num_table_entries];
    in_facet_intersection_bits = new VBITS_TYPE[num_table_entries];
    is_table_allocated = true;

    ComputeInRidgeBits();
    ComputeInFacetIntersectionBits();
  }


  /// Check VBITS_TYPE.
  template <typename DTYPE, typename VBITS_TYPE, typename NUM_TYPE>
  template  <typename DTYPE2>
  void FACET_INTERSECTION_TABLE<DTYPE,VBITS_TYPE,NUM_TYPE>::
  CheckVertexBitsType(const DTYPE2 dimension) const
  {
    NUM_TYPE num_vert = IJK::compute_num_cube_vertices(dimension);

    if (!check_number_of_bits<VBITS_TYPE>(num_vert)) {
      IJK::ERROR error;

      error.AddMessage
        ("Programming error using template class FACET_INTERSECTION_TABLE.");
      error.AddMessage
        ("  Fewer than ", num_vert, 
         " bits in type template parameter VBITS_TYPE.");
      error.AddMessage
        ("  Replace type template parameter with type with more bits.");
      throw error;
    }
  }

  /// Check NUM_TYPE.
  template <typename DTYPE, typename VBITS_TYPE, typename NUM_TYPE>
  template  <typename DTYPE2>
  void FACET_INTERSECTION_TABLE<DTYPE,VBITS_TYPE,NUM_TYPE>::
  CheckNumType(const DTYPE2 dimension) const
  {

    NUM_TYPE num_facets = IJK::compute_num_cube_facets(dimension);

    if (!check_number_of_bits<VBITS_TYPE>(num_facets)) {
      IJK::ERROR error;

      error.AddMessage
        ("Programming error using template class FACET_INTERSECTION_TABLE.");
      error.AddMessage
        ("  Fewer than ", num_facets,
         " bits in type template parameter NUM_TYPE.");
      error.AddMessage
        ("  Replace type template parameter with type with more bits.");

      throw error;
    }
  }
  

}

#endif

