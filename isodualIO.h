/// \file isodualIO.h
/// IO classes and routines for isodual.

/*
  IJK: Isosurface Jeneration Kode
  Copyright (C) 2011-2017 Rephael Wenger

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

#ifndef _ISODUALIO_
#define _ISODUALIO_

#include <ctime>
#include <string>

#include "ijk.txx"
#include "ijkgrid_nrrd.txx"
#include "ijkstring.txx"

#include "ijkdualIO.txx"

#include "isodual_types.h"
#include "isodual_datastruct.h"


namespace ISODUAL {

  // **************************************************
  // TYPE DEFINITIONS
  // **************************************************

  typedef float COLOR_TYPE;           //!< Color type.

  //! Nrrd header.
  typedef IJK::NRRD_DATA<int, AXIS_SIZE_TYPE> NRRD_HEADER; 

  typedef enum { OFF, IV, PLY } OUTPUT_FORMAT;    //!< Output format.


  // **************************************************
  // IO INFORMATION
  // **************************************************

  //! IO information
  class IO_INFO:public IJKDUAL::DUALISO_DATA_FLAGS {

  protected:
    void Init();

  public:
    SCALAR_ARRAY isovalue;        ///< List of isovalues.
    std::vector<std::string> isovalue_string;
    COORD_ARRAY grid_spacing;
    std::string input_filename;
    std::string output_filename;
    std::string output_filename_prefix;
    bool label_with_isovalue;
    std::string output_off_filename;
    std::string output_ply_filename;
    std::string output_iv_filename;
    bool are_output_filenames_set;
    std::string isotable_directory;
    bool flag_output_off;    ///< Output Geomview .off file.
    bool flag_output_ply;    ///< Output PLY file.
    bool flag_output_iv;     ///< Output OpenInventor file.
    bool flag_report_time;
    bool flag_report_info;
    bool flag_use_stdout;
    bool flag_nowrite;
    bool flag_silent;
    bool flag_no_warn;
    bool flag_subsample;
    bool flag_report_all_isov;
    std::string report_isov_filename;
    int subsample_resolution;
    bool flag_supersample;
    int supersample_resolution;
    bool flag_color_alternating;  ///< Color simplices in alternating cubes
    int region_length;

    /// Color isosurface boundary vertices.
    bool flag_color_vert;         

    /// List of high resolution arguments,
    ///   e.g., "-highres {coord list}".
    std::vector<std::string> high_resolution_option;

    /// Flag for options set.
    bool is_qei_method_set;
    bool is_tri4_position_method_set;
    bool is_file_format_set;

    /// Return number of selected output formats.
    int NumOutputFormats() const;

    /// Copy io_info.
    void Set(const IO_INFO & io_info);

    /// Set output format to output_format.
    /// - Note: More than one output format may be set.
    void SetOutputFormat(const OUTPUT_FORMAT output_format);

    /// Set output filename for output format which is flagged true.
    /// @pre At most one output format is flagged true.
    void SetOutputFilename(const char * output_filename);

    /// Set output filename for output format which is flagged true.
    /// @pre At most one output format is flagged true.
    void SetOutputFilename(const std::string & output_filename)
    { SetOutputFilename(output_filename.c_str()); }

    /// Set output filename for a given output format.
    void SetOutputFilename
    (const OUTPUT_FORMAT output_format, const char * output_filename);

    /// Set output filename for a given output format.
    void SetOutputFilename
    (const OUTPUT_FORMAT output_format, const std::string & output_filename)
    { SetOutputFilename(output_format, output_filename.c_str()); }

    /// Construct output filenames.
    /// @param i Construct filenames for isovalue i.
    void ConstructOutputFilenames(const int i);


  public:
    IO_INFO() { Init(); };
    ~IO_INFO() { Init(); };
  };


  // **************************************************
  // OUTPUT INFORMATION
  // **************************************************

  /// Output information.
  class OUTPUT_INFO:public IO_INFO {

  protected:
    void Init();

  public:
    typedef ISODUAL::COLOR_TYPE COLOR_TYPE;
    
  public:
    int dimension;
    int num_vertices_per_isopoly;
    SCALAR_TYPE isovalue[2];
    COORD_ARRAY grid_spacing;
    int grow_factor;
    int shrink_factor;

    OUTPUT_INFO() { Init(); };
    ~OUTPUT_INFO() { Init(); };

    void SetDimension(const int d);
  };

  // **************************************************
  // TIMING FUNCTIONS/CLASSES
  // **************************************************

  /// Elapsed CPU time.
  class ELAPSED_CPU_TIME {

  protected:
    clock_t t;

  public:
    ELAPSED_CPU_TIME() { t = clock(); };

    clock_t getElapsed() {
      clock_t old_t = t;
      t = clock();
      return(t - old_t);
    };
  };

  /// Elapsed wall time.
  class ELAPSED_TIME {

  protected:
    time_t t;

  public:
    ELAPSED_TIME() { time(&t);  };

    double getElapsed() {
      time_t old_t = t;
      time(&t);
      return(difftime(t,old_t));
    };
  };

  /// IO time.
  struct IO_TIME {
    double read_nrrd_time;  ///< Wall time to read nrrd file.
    double write_time;      ///< Wall time to write output.
  };

  // **************************************************
  // PARSE COMMAND LINE
  // **************************************************

  /// Parse the command line.
  void parse_command_line(int argc, char **argv, IO_INFO & io_info);

  /// Check input information in io_info
  bool check_input
    (const IO_INFO & io_info, 
     const DUALISO_SCALAR_GRID_BASE & scalar_grid,
     IJK::ERROR & error);

  // **************************************************
  // READ NEARLY RAW RASTER DATA (nrrd) FILE
  // **************************************************

  /// Read a nearly raw raster data (nrrd) file.
  void read_nrrd_file
    (const char * input_filename, DUALISO_SCALAR_GRID & scalar_grid, 
     NRRD_HEADER & nrrd_header, IO_TIME & io_time);

  /// Read a nearly raw raster data (nrrd) file.
  /// Pass C++ string instead of char * input filename.
  void read_nrrd_file
  (const std::string input_filename, DUALISO_SCALAR_GRID & scalar_grid, 
   NRRD_HEADER & nrrd_header, IO_TIME & io_time);


  // **************************************************
  // RESCALE ROUTINES
  // **************************************************

  /// Rescale vertex coordinates by grid_spacing.
  void rescale_vertex_coord
    (const int dimension, const COORD_TYPE * grid_spacing,
     std::vector<COORD_TYPE> & vertex_coord);

  /// Rescale vertex coordinates by grid_spacing.
  /// @pre grid_spacing.size() equals vertex dimension.
  void rescale_vertex_coord(const std::vector<COORD_TYPE> & grid_spacing,
                            std::vector<COORD_TYPE> & vertex_coord);

  /// Rescale subsampled/supersampled vertex coordinates.
  void rescale_vertex_coord
  (const int grow_factor, const int shrink_factor, COORD_ARRAY & vertex_coord);

  /// Rescale vertex coordinates by grow and shrink factor and by grid_spacing.
  /// @pre grid_spacing.size() equals vertex dimension.
  void rescale_vertex_coord
    (const int grow_factor, const int shrink_factor,
     const COORD_ARRAY & grid_spacing, COORD_ARRAY & vertex_coord);


  // **************************************************
  // WRITE_DUAL_MESH
  // **************************************************

  /// Write dual mesh with output format output_format.
  /// @param output_format Output format.
  void write_dual_mesh
  (const OUTPUT_INFO & output_info, const OUTPUT_FORMAT output_format,
   const std::vector<COORD_TYPE> & vertex_coord, 
   const std::vector<VERTEX_INDEX> & plist);

  /// Write dual mesh.
  void write_dual_mesh
    (const OUTPUT_INFO & output_info,
     const std::vector<COORD_TYPE> & vertex_coord, 
     const std::vector<VERTEX_INDEX> & slist);

  /// Write dual mesh and record output time.
  void write_dual_mesh
    (const OUTPUT_INFO & output_info,
     const std::vector<COORD_TYPE> & vertex_coord, 
     const std::vector<VERTEX_INDEX> & slist,
     IO_TIME & io_time);

  /// Write dual mesh and color facets with output format output_format.
  void write_dual_mesh_color
  (const OUTPUT_INFO & output_info, const OUTPUT_FORMAT output_format,
   const std::vector<COORD_TYPE> & vertex_coord, 
   const std::vector<VERTEX_INDEX> & plist,
   const COLOR_TYPE * front_color, const COLOR_TYPE * back_color);

  /// Write dual mesh and color facets.
  void write_dual_mesh_color
    (const OUTPUT_INFO & output_info,
     const std::vector<COORD_TYPE> & vertex_coord, 
     const std::vector<VERTEX_INDEX> & slist,
     const COLOR_TYPE * front_color, const COLOR_TYPE * back_color);

  /// Write dual mesh, color facets, and record output time.
  void write_dual_mesh_color
    (const OUTPUT_INFO & output_info,
     const std::vector<COORD_TYPE> & vertex_coord, 
     const std::vector<VERTEX_INDEX> & slist, 
     const COLOR_TYPE * front_color, const COLOR_TYPE * back_color,
     IO_TIME & io_time);

  /// Write dual isosurface triangular mesh
  /// @param output_info Output information.
  /// @param output_format Output format.
  /// @param vertex_coord List of vertex coordinates.
  /// @param tri_vert[] List of triangle vertices.
  ///        tri_vert[3*i+k] is k'th vertex of triangle i.
  void write_dual_tri_mesh
  (const OUTPUT_INFO & output_info, const OUTPUT_FORMAT output_format,
   const std::vector<COORD_TYPE> & vertex_coord,
   const std::vector<VERTEX_INDEX> & tri_vert);

  /// Write dual isosurface triangular mesh.
  /// @param output_info Output information.
  /// @param vertex_coord List of vertex coordinates.
  /// @param tri_vert[] List of triangle vertices.
  ///        tri_vert[3*i+k] is k'th vertex of triangle i.
  void write_dual_tri_mesh
  (const OUTPUT_INFO & output_info,
   const std::vector<COORD_TYPE> & vertex_coord,
   const std::vector<VERTEX_INDEX> & tri_vert);

  /// Write dual isosurface triangular mesh.
  /// Record write time.
  void write_dual_tri_mesh
  (const OUTPUT_INFO & output_info,
   const std::vector<COORD_TYPE> & vertex_coord,
   const std::vector<VERTEX_INDEX> & tri_vert,
   IO_TIME & io_time);

  /// Write dual isosurface mesh of quad and triangles.
  /// @param output_info Output information.
  /// @param output_format Output format.
  /// @param vertex_coord List of vertex coordinates.
  void write_dual_quad_tri_mesh
  (const OUTPUT_INFO & output_info, const OUTPUT_FORMAT output_format,
   const std::vector<COORD_TYPE> & vertex_coord,
   const std::vector<VERTEX_INDEX> & quad_vert,
   const std::vector<VERTEX_INDEX> & tri_vert);

  /// Write dual isosurface mesh of quad and triangles.
  /// @param output_info Output information.
  /// @param vertex_coord List of vertex coordinates.
  void write_dual_quad_tri_mesh
  (const OUTPUT_INFO & output_info,
   const std::vector<COORD_TYPE> & vertex_coord,
   const std::vector<VERTEX_INDEX> & quad_vert,
   const std::vector<VERTEX_INDEX> & tri_vert);

  /// Write dual isosurface mesh of quad and triangles and record output time.
  void write_dual_quad_tri_mesh
  (const OUTPUT_INFO & output_info,
   const std::vector<COORD_TYPE> & vertex_coord,
   const std::vector<VERTEX_INDEX> & quad_vert,
   const std::vector<VERTEX_INDEX> & tri_vert,
   IO_TIME & io_time);

  /// Write dual isosurface triangular mesh and color vertices
  ///   with output format output_format.
  /// @param output_info Output information.
  /// @param output_format Output format.
  /// @param vertex_coord List of vertex coordinates.
  /// @param tri_vert[] List of triangle vertices.
  ///        tri_vert[3*i+k] is k'th vertex of triangle i.
  void write_dual_tri_mesh_color_vertices
  (const OUTPUT_INFO & output_info, const OUTPUT_FORMAT output_format,
   const std::vector<COORD_TYPE> & vertex_coord,
   const std::vector<VERTEX_INDEX> & tri_vert,
   const COLOR_TYPE * front_color, const COLOR_TYPE * back_color);

  /// Write dual isosurface triangular mesh, color vertices.
  /// @param output_info Output information.
  /// @param vertex_coord List of vertex coordinates.
  /// @param tri_vert[] List of triangle vertices.
  ///        tri_vert[3*i+k] is k'th vertex of triangle i.
  void write_dual_tri_mesh_color_vertices
  (const OUTPUT_INFO & output_info,
   const std::vector<COORD_TYPE> & vertex_coord,
   const std::vector<VERTEX_INDEX> & tri_vert,
   const COLOR_TYPE * front_color, const COLOR_TYPE * back_color);

  /// Write dual isosurface triangular mesh, color vertices 
  ///   and record output time.
  void write_dual_tri_mesh_color_vertices
  (const OUTPUT_INFO & output_info,
   const std::vector<COORD_TYPE> & vertex_coord,
   const std::vector<VERTEX_INDEX> & tri_vert,
   const COLOR_TYPE * front_color, const COLOR_TYPE * back_color,
   IO_TIME & io_time);

  /// Write dual isosurface mesh of quad and triangles and color vertices
  ///   with output format output_format.
  /// @param output_info Output information.
  /// @param output_format Output format.
  /// @param vertex_coord List of vertex coordinates.
  void write_dual_quad_tri_mesh_color_vertices
  (const OUTPUT_INFO & output_info, const OUTPUT_FORMAT output_format,
   const std::vector<COORD_TYPE> & vertex_coord,
   const std::vector<VERTEX_INDEX> & quad_vert,
   const std::vector<VERTEX_INDEX> & tri_vert,
   const COLOR_TYPE * front_color, const COLOR_TYPE * back_color);

  /// Write dual isosurface mesh of quad and triangles and color vertices.
  /// @param output_info Output information.
  /// @param vertex_coord List of vertex coordinates.
  void write_dual_quad_tri_mesh_color_vertices
  (const OUTPUT_INFO & output_info,
   const std::vector<COORD_TYPE> & vertex_coord,
   const std::vector<VERTEX_INDEX> & quad_vert,
   const std::vector<VERTEX_INDEX> & tri_vert,
   const COLOR_TYPE * front_color, const COLOR_TYPE * back_color);

  /// Write dual isosurface mesh of quad and triangles, color vertices,
  ///   and report output time.
  void write_dual_quad_tri_mesh_color_vertices
  (const OUTPUT_INFO & output_info,
   const std::vector<COORD_TYPE> & vertex_coord,
   const std::vector<VERTEX_INDEX> & quad_vert,
   const std::vector<VERTEX_INDEX> & tri_vert,
   const COLOR_TYPE * front_color, const COLOR_TYPE * back_color,
   IO_TIME & io_time);


  // **************************************************
  // SET ROUTINES
  // **************************************************

  /// Set dualiso_data based on io_info.
  /// Precondition: Scalar field in dualiso_data must be set before
  ///   this routines is called.
  void set_dualiso_data
  (const IO_INFO & io_info, DUALISO_DATA & dualiso_data, 
   DUALISO_TIME & dualiso_time);

  /// Set output_info based on isotable, io_info and isovalue index i.
  void set_output_info
  (const IO_INFO & io_info, 
   const int i, OUTPUT_INFO & output_info);

  /// Set simplices in alternating cubes to have different colors.
  void set_color_alternating
  (const DUALISO_GRID & grid, const std::vector<VERTEX_INDEX> & cube_list, 
   COLOR_TYPE * color);


  // **************************************************
  // REPORT SCALAR FIELD OR ISOSURFACE INFORMATION
  // **************************************************

  void report_num_cubes
    (const DUALISO_GRID & full_grid, const IO_INFO & io_info, 
     const DUALISO_DATA & dualiso_data);

  void report_num_cubes
    (const DUALISO_GRID & full_grid, const IO_INFO & io_info, 
     const DUALISO_GRID & dualiso_data_grid);

  void warn_non_manifold(const IO_INFO & io_info);


  // **************************************************
  // REPORT TIMING INFORMATION
  // **************************************************

  void report_dualiso_time
    (const IO_INFO & io_info, const DUALISO_TIME & dualiso_time, 
     const char * mesh_type_string);

  void report_time
    (const IO_INFO & io_info, const IO_TIME & io_time, 
     const DUALISO_TIME & dualiso_time, const double total_elapsed_time);


  // **************************************************
  // USAGE/HELP MESSAGES
  // **************************************************

  void usage_error();
  void usage(std::ostream & out, const int return_code);
  void usage_all(std::ostream & out, const int return_code);
  void help(), help_all();

}

#endif
