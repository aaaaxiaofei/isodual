/// \file isodualIO.cxx
/// IO routines for isodual

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

#include <assert.h>
#include <time.h>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>

#include "ijkcommand_line.txx"
#include "ijkIO.txx"
#include "ijkmesh.txx"
#include "ijkstring.txx"
#include "ijkprint.txx"

#include "isodualIO.h"

using namespace IJK;
using namespace IJKDUAL;
using namespace ISODUAL;

using namespace std;

// **************************************************
// PARSE COMMAND LINE
// **************************************************

// local namespace
namespace {

  typedef enum
    {SUBSAMPLE_OPT, SUPERSAMPLE_OPT, POSITION_OPT, CUBE_CENTER_OPT,
     MANIFOLD_OPT, MULTI_ISOV_OPT, SINGLE_ISOV_OPT, 
     SELECT_SPLIT_OPT, CONNECT_AMBIG_OPT,
     SEP_NEG_OPT, SEP_POS_OPT,
     TRIMESH_OPT, TRIMESH_UNIFORM_OPT,
     TRIMESH_SPLIT_MAX_OPT, TRIMESH_MAX_ANGLE_OPT,
     TRIMESH_ONLY_TRI4_OPT, TRIMESH_TRI4_BY_DISTANCE_OPT,
     TRIMESH_TRI4_MAX_ANGLE_OPT,
     TRI4_OPT,
     TRI4_CENTROID_OPT,
     QEI_INTERPOLATE_SCALAR_OPT, QEI_INTERPOLATE_COORD_OPT, 
     QEI_AVERAGE_OPT,
     COLOR_VERT_OPT,
     HELP_OPT, HELP_ALL_OPT, USAGE_OPT, ALL_OPTIONS_OPT,
     OFF_OPT, IV_OPT, PLY_OPT,
     OUTPUT_FILENAME_OPT, OUTPUT_FILENAME_PREFIX_OPT, STDOUT_OPT, 
     LABEL_WITH_ISOVALUE_OPT,
     NO_WRITE_OPT, SILENT_OPT, NO_WARN_OPT,
     INFO_OPT, TIME_OPT, OUT_ISOV_OPT, UNKNOWN_OPT} OPTION_TYPE;

  typedef enum {
    REGULAR_OPTG, EXTENDED_OPTG, QDUAL_OPTG, TESTING_OPTG
  } OPTION_GROUP;

  typedef IJK::SUFFIX_TYPE_LIST<OUTPUT_FORMAT> OUTPUT_FILE_TYPE_LIST;
  OUTPUT_FILE_TYPE_LIST output_file_type_list;

  IJK::COMMAND_LINE_OPTIONS<OPTION_TYPE,OPTION_GROUP> options;
};

// local namespace
namespace {

  INTERPOLATION_TYPE get_interpolation_type(char * s)
  // convert string s into parameter token
  {
    INTERPOLATION_TYPE type = LINEAR_INTERPOLATION;

    if (strcmp(s, "linear") == 0) 
      { type = LINEAR_INTERPOLATION; }
    else if (strcmp(s, "multilinear") == 0) 
      { type = MULTILINEAR_INTERPOLATION; }
    else {
      cerr << "Error in input parameter -interpolate.  Illegal interpolation type: " 
           << s << "." << endl;
      exit(1030);
    }

    return(type);
  }

  VERTEX_POSITION_METHOD get_vertex_position_method(char * s)
  // convert string s into parameter token
  {
    VERTEX_POSITION_METHOD method = CENTROID_EDGE_ISO;
    string str = s;

    if (str == "cube_center") {
      method = CUBE_CENTER;
    }
    else if (str == "centroid") {
      method = CENTROID_EDGE_ISO;
    }
    else if (str == "diagonal") {
      method = DIAGONAL_INTERPOLATION;
    }
    else if (str == "qdual") {
      method = QDUAL_INTERPOLATION;
    }
    else if (str == "ivol_lifted02") {
      method = IVOL_LIFTED02;
    }
    else {
      cerr << "Error in input parameter -position.  Illegal position method: " 
           << str << "." << endl;
      exit(1030);
    }

    return(method);
  }

}

namespace {

  void set_command_line_options()
  {
    int iarg;

    options.SetLabelWidth(15);
    options.AddLabelTab(20);

    // Regular options.
    options.AddUsageOptionBeginOr(REGULAR_OPTG);

    options.AddOption1Arg
      (SUBSAMPLE_OPT, "SUBSAMPLE_OPT", REGULAR_OPTG,
       "-subsample", "S", "Subsample grid at every S vertices.");
    options.AddToHelpMessage
      (SUBSAMPLE_OPT, "S must be an integer greater than 1.");

    options.AddOption1Arg
      (SUPERSAMPLE_OPT, "SUPERSAMPLE_OPT", REGULAR_OPTG,
       "-supersample", "S", 
       "Supersample grid at every S vertices.");
    options.AddToHelpMessage
      (SUPERSAMPLE_OPT, "S must be an integer greater than 1.");

    options.AddUsageOptionEndOr(REGULAR_OPTG);
    options.AddUsageOptionNewline(REGULAR_OPTG);
    options.AddUsageOptionBeginOr(REGULAR_OPTG);

    options.AddOption1Arg
      (POSITION_OPT, "POSITION_OPT", REGULAR_OPTG,
       "-position", "{centroid|cube_center}",
       "Isosurface vertex position method.");
    iarg = options.AddArgChoice
      (POSITION_OPT, "centroid", 
       "Position isosurface vertices at centroid of");
    options.AddToHelpArgMessage
      (POSITION_OPT, iarg,
       "intersection of grid edges and isosurface.");
    iarg = options.AddArgChoice
      (POSITION_OPT, "cube_center",
       "Position isosurface vertices at cube_centers.");

    options.AddOptionNoArg
      (CUBE_CENTER_OPT, "CUBE_CENTER_OPT", REGULAR_OPTG, "-cube_center",
       "Position isosurface vertices at cube_centers.");
    options.AddToHelpMessage
      (CUBE_CENTER_OPT, "Equivalent to \"-position cube_center\".");

    options.AddUsageOptionEndOr(REGULAR_OPTG);
    options.AddUsageOptionNewline(REGULAR_OPTG);

    options.AddUsageOptionBeginOr(REGULAR_OPTG);
    options.AddOptionNoArg
      (MANIFOLD_OPT, "MANIFOLD_OPT", REGULAR_OPTG, "-manifold",
       "Isosurface mesh has a manifold representation (Default).");
    options.AddToHelpMessage
      (MANIFOLD_OPT, "Allows multiple isosurface vertices per cube and",
       "splits isosurface vertices to avoid non-manifold edges.");

    options.AddOptionNoArg
      (MULTI_ISOV_OPT, "MULTI_ISOV_OPT", REGULAR_OPTG, "-multi_isov", 
       "Allow multiple isosurface vertices per cube but");
    options.AddToHelpMessage
      (MULTI_ISOV_OPT, 
       "don't change isosurface mesh to avoid non-manifold edges.",
       "Isosurface may have non-manifold edges/vertices.");

    options.AddOptionNoArg
      (SINGLE_ISOV_OPT, "SINGLE_ISOV_OPT", REGULAR_OPTG, "-single_isov", 
       "Single isosurface vertex per cube.");
    options.AddToHelpMessage
      (SINGLE_ISOV_OPT, "Isosurface may have non-manifold edges/vertices.");

    options.AddUsageOptionEndOr(REGULAR_OPTG);

    options.AddUsageOptionNewline(REGULAR_OPTG);
    options.AddUsageOptionBeginOr(REGULAR_OPTG);

    options.AddOptionNoArg
      (TRIMESH_OPT, "TRIMESH_OPT", REGULAR_OPTG, "-trimesh", 
       "Triangulate all isosurface quadrilaterals.");
    options.AddToHelpMessage
      (TRIMESH_OPT, 
       "All triangulation options are allowed only",
       "with 3D scalar data.");

    options.AddOptionNoArg
      (TRIMESH_UNIFORM_OPT, "TRIMESH_UNIFORM_OPT", REGULAR_OPTG,
       "-trimesh_uniform", "Triangulate all isosurface quadrilaterals.");
    options.AddToHelpMessage
      (TRIMESH_UNIFORM_OPT, "Uniformly split quadrilaterals.");

    options.AddOptionNoArg
      (TRIMESH_TRI4_MAX_ANGLE_OPT, "TRIMESH_TRI4_MAX_ANGLE_OPT", REGULAR_OPTG,
       "-trimesh_tri4_max_angle", 
       "Triangulate isosurface quadrilaterals into two");
    options.AddToHelpMessage
      (TRIMESH_TRI4_MAX_ANGLE_OPT,
       "or four triangles.",
       "Choose triangulation which maximizes the minimum angle.");
    options.AddSynonym(TRIMESH_TRI4_MAX_ANGLE_OPT, "-tri4");

    options.AddUsageOptionEndOr(REGULAR_OPTG);
    options.AddUsageOptionNewline(REGULAR_OPTG);

    options.AddUsageOptionBeginOr(REGULAR_OPTG);
    options.AddOptionNoArg
      (OFF_OPT, "OFF_OPT", REGULAR_OPTG, "-off", 
       "Output in geomview OFF format. (Default.)");

    options.AddOptionNoArg
      (PLY_OPT, "PLY_OPT", REGULAR_OPTG, "-ply", 
       "Output in Stanford Polygon File Format (PLY).");
    options.AddToHelpMessage
      (PLY_OPT, "(Allowed only with 3D scalar data.)");
    options.AddUsageOptionEndOr(REGULAR_OPTG);

    options.AddUsageOptionNewline(REGULAR_OPTG);

    options.AddUsageOptionBeginOr(REGULAR_OPTG);
    options.AddOption1Arg
      (OUTPUT_FILENAME_OPT, "OUTPUT_FILENAME_OPT", REGULAR_OPTG, 
       "-o", "{output_filename}", 
       "Write isosurface to file {output_filename}.");

    options.AddOption1Arg
      (OUTPUT_FILENAME_PREFIX_OPT, "OUTPUT_FILENAME_PREFIX_OPT", REGULAR_OPTG, 
       "-prefix", "{prefix}", 
       "Write isosurface to files with prefix {prefix}.");

    options.AddOptionNoArg
      (STDOUT_OPT, "STDOUT_OPT", REGULAR_OPTG, 
       "-stdout", "Write isosurface to standard output.");

    options.AddUsageOptionEndOr(REGULAR_OPTG);

    options.AddOptionNoArg
      (LABEL_WITH_ISOVALUE_OPT, "LABEL_WITH_ISOVALUE_OPT", REGULAR_OPTG,
       "-label_with_isovalue", "Include isovalue in output file name.");

    options.AddUsageOptionNewline(REGULAR_OPTG);

    options.AddOptionNoArg
      (SILENT_OPT, "SILENT_OPT", REGULAR_OPTG, "-silent", 
       "Silent mode.  No output messages.");
    options.AddSynonym(SILENT_OPT, "-s");

    options.AddOptionNoArg
      (NO_WARN_OPT, "NO_WARN_OPT", REGULAR_OPTG, "-no_warn", 
       "Do not output non-manifold warning.");

    options.AddOptionNoArg
      (NO_WRITE_OPT, "NO_WRITE_OPT", REGULAR_OPTG, "-no_write", 
       "Don't wite isosurface.");

    options.AddOptionNoArg
      (TIME_OPT, "TIME_OPT", REGULAR_OPTG, "-time", "Output running time.");

    options.AddOptionNoArg
      (INFO_OPT, "INFO_OPT", REGULAR_OPTG, "-info", 
       "Output more information about isosurface.");

    options.AddUsageOptionNewline(REGULAR_OPTG);


    options.AddOptionNoArg
      (USAGE_OPT, "USAGE_OPT", REGULAR_OPTG, "-usage", 
       "Print usage message.");

    options.AddOptionNoArg
      (ALL_OPTIONS_OPT, "ALL_OPTIONS_OPT", REGULAR_OPTG, "-all_options", 
       "Print usage message with all options.");


    options.AddOptionNoArg
      (HELP_OPT, "HELP_OPT", REGULAR_OPTG, "-help",
       "Print help message for standard options.");
    options.AddSynonym(HELP_OPT, "-h");

    options.AddOptionNoArg
      (HELP_ALL_OPT, "HELP_ALL_OPT", REGULAR_OPTG, "-help_all", 
       "Print help message for all options.");

    options.AddUsageOptionNewline(REGULAR_OPTG);


    // Extended options.
    options.AddUsageOptionBeginOr(EXTENDED_OPTG);

    options.AddOptionNoArg
      (SEP_NEG_OPT, "SEP_NEG_OPT", EXTENDED_OPTG, "-sep_neg", 
       "Isosurface patches separate negative grid vertices.");

    options.AddOptionNoArg
      (SEP_POS_OPT, "SEP_POS_OPT", EXTENDED_OPTG, "-sep_pos", 
       "Isosurface patches separate positive grid vertices.");

    options.AddUsageOptionEndOr(EXTENDED_OPTG);

    options.AddOptionNoArg
      (SELECT_SPLIT_OPT, "SELECT_SPLIT_OPT", EXTENDED_OPTG, "-select_split", 
       "Select which cube has configuration of split isosurface");
    options.AddToHelpMessage
      (SELECT_SPLIT_OPT, 
       "vertices where adjacent cubes share an ambiguous facet.");

    options.AddOptionNoArg
      (CONNECT_AMBIG_OPT, "CONNECT_AMBIG_OPT", EXTENDED_OPTG, 
       "-connect_ambig", "Select connection of ambiguous cubes to increase");
    options.AddToHelpMessage
      (CONNECT_AMBIG_OPT, "connectivity of the isosurface."); 

    options.AddUsageOptionNewline(EXTENDED_OPTG);
    options.AddUsageOptionBeginOr(EXTENDED_OPTG);

    options.AddOptionNoArg
      (TRIMESH_SPLIT_MAX_OPT, "TRIMESH_SPLIT_MAX_OPT", EXTENDED_OPTG,
       "-trimesh_split_max", 
       "Triangulate all isosurface quadrilaterals.");
    options.AddToHelpMessage
      (TRIMESH_SPLIT_MAX_OPT, "Split max quadrilateral angle.");

    options.AddOptionNoArg
      (TRIMESH_MAX_ANGLE_OPT, "TRIMESH_MAX_ANGLE_OPT", EXTENDED_OPTG,
       "-trimesh_max_angle", 
       "Triangulate all isosurface quadrilaterals.");
    options.AddToHelpMessage
    (TRIMESH_MAX_ANGLE_OPT, 
     "Choose triangulation which maximizes the minimum",
     "triangle angle.");

    options.AddOptionNoArg
      (TRIMESH_ONLY_TRI4_OPT, "TRIMESH_ONLY_TRI4_OPT", EXTENDED_OPTG, 
       "-trimesh_only_tri4", 
       "Triangulate all isosurface quadrilaterals");
    options.AddToHelpMessage
      (TRIMESH_ONLY_TRI4_OPT, "into four triangles.");

    options.AddUsageOptionNewline(EXTENDED_OPTG);

    options.AddOptionNoArg
      (TRIMESH_TRI4_BY_DISTANCE_OPT, "TRIMESH_TRI4_BY_DISTANCE_OPT",
       EXTENDED_OPTG, "-trimesh_tri4_by_distance", 
       "Triangulate isosurface quadrilaterals into two");
    options.AddToHelpMessage
      (TRIMESH_TRI4_BY_DISTANCE_OPT,
       "or four triangles.  Triangulate into four triangles",
       "if all quadrilateral vertices are far from cube facets.");

    options.AddUsageOptionEndOr(EXTENDED_OPTG);
    options.AddUsageOptionNewline(EXTENDED_OPTG);

    options.AddOptionNoArg
      (TRI4_CENTROID_OPT, "TRI4_CENTROID_OPT", EXTENDED_OPTG, "-tri4_centroid",
       "Position additional triangulation vertex at centroid");
    options.AddToHelpMessage
      (TRI4_CENTROID_OPT, "of quadrilateral vertices.");

    options.AddUsageOptionNewline(EXTENDED_OPTG);
    options.AddUsageOptionBeginOr(EXTENDED_OPTG);

    options.AddOptionNoArg
      (QEI_INTERPOLATE_SCALAR_OPT, "QEI_INTERPOLATE_SCALAR_OPT", 
       EXTENDED_OPTG, "-qei_interpolate_scalar", 
       "Compute intersection of isosurface quadrilaterals and");
    options.AddToHelpMessage
      (QEI_INTERPOLATE_SCALAR_OPT,
       "grid edges using linear interpolation of scalar values",
       "at grid edge endpoints.");

    options.AddOptionNoArg
      (QEI_INTERPOLATE_COORD_OPT, "QEI_INTERPOLATE_COORD_OPT", 
       EXTENDED_OPTG, "-qei_interpolate_coord", 
       "Compute intersection of isosurface quadrilaterals and");
    options.AddToHelpMessage
      (QEI_INTERPOLATE_COORD_OPT,
       "grid edges using linear interpolation of quadrilateral",
       "vertex coordinates.");

    options.AddOptionNoArg
      (QEI_AVERAGE_OPT, "QEI_AVERAGE_OPT", EXTENDED_OPTG,
       "-qei_average", 
       "Compute intersection of isosurface quadrilaterals and");
    options.AddToHelpMessage
      (QEI_AVERAGE_OPT,
       "grid edges using the average of the projection",
       "of quadrilateral vertices on the grid edge.");

    options.AddUsageOptionEndOr(EXTENDED_OPTG);
    options.AddUsageOptionNewline(EXTENDED_OPTG);


    options.AddOption1Arg
      (OUT_ISOV_OPT, "OUT_ISOV_OPT", EXTENDED_OPTG, 
       "-out_isov", "{output_filename}", 
       "Write information about isosurface vertices");
    options.AddToHelpMessage
      (OUT_ISOV_OPT, "to file {output_filename}.");
  }

};


void create_output_file_type_list(OUTPUT_FILE_TYPE_LIST & list)
{
  list.clear();
  list.push_back(make_pair(OFF, ".off"));
  list.push_back(make_pair(IV, ".iv"));
  list.push_back(make_pair(PLY, ".ply"));
}


// Return true if option OPTA is processed (i.e., matches some case in
//   switch statement.)
template <typename IO_INFO_TYPE>
bool process_option
(const OPTION_TYPE optA, const int argc, char **argv, 
 int & iarg, IO_INFO_TYPE & io_info)
{
  IJK::ERROR error;

  switch(optA) {

  case SUBSAMPLE_OPT:
    io_info.subsample_resolution = get_arg_int(iarg, argc, argv, error);
    io_info.flag_subsample = true;
    iarg++;
    break;

  case SUPERSAMPLE_OPT:
    io_info.supersample_resolution = get_arg_int(iarg, argc, argv, error);
    io_info.flag_supersample = true;
    iarg++;
    break;

  case POSITION_OPT:
    iarg++;
    if (iarg >= argc) usage_error();
    io_info.vertex_position_method = 
      get_vertex_position_method(argv[iarg]);
    break;

  case CUBE_CENTER_OPT:
    io_info.vertex_position_method = CUBE_CENTER;
    break;

  case MANIFOLD_OPT:
    io_info.allow_multiple_iso_vertices = true;
    io_info.flag_split_non_manifold = true;
    break;

  case MULTI_ISOV_OPT:
    io_info.allow_multiple_iso_vertices = true;
    io_info.flag_split_non_manifold = false;
    break;

  case SINGLE_ISOV_OPT:
    io_info.allow_multiple_iso_vertices = false;
    break;

  case SELECT_SPLIT_OPT:
    io_info.allow_multiple_iso_vertices = true;
    io_info.flag_select_split = true;
    break;

  case CONNECT_AMBIG_OPT:
    io_info.flag_connect_ambiguous = true;
    io_info.allow_multiple_iso_vertices = true;
    break;

  case SEP_NEG_OPT:
    io_info.allow_multiple_iso_vertices = true;
    io_info.flag_separate_neg = true;
    break;

  case SEP_POS_OPT:
    io_info.allow_multiple_iso_vertices = true;
    io_info.flag_separate_neg = false;
    break;

  case TRIMESH_OPT:
    io_info.use_triangle_mesh = true;
    break;

  case TRIMESH_SPLIT_MAX_OPT:
    io_info.use_triangle_mesh = true;
    io_info.quad_tri_method = SPLIT_MAX_ANGLE;
    break;

  case TRIMESH_MAX_ANGLE_OPT:
    io_info.use_triangle_mesh = true;
    io_info.quad_tri_method = MAX_MIN_ANGLE;
    break;

  case TRIMESH_ONLY_TRI4_OPT:
    io_info.use_triangle_mesh = true;
    io_info.quad_tri_method = ONLY_TRI4;
    io_info.flag_tri4_quad = true;
    break;

  case TRIMESH_TRI4_BY_DISTANCE_OPT:
    io_info.use_triangle_mesh = true;
    io_info.quad_tri_method = TRI4_BY_DISTANCE;
    io_info.flag_tri4_quad = true;
    break;

  case TRIMESH_TRI4_MAX_ANGLE_OPT:
  case TRI4_OPT:
    io_info.use_triangle_mesh = true;
    io_info.quad_tri_method = TRI4_MAX_MIN_ANGLE;
    io_info.flag_tri4_quad = true;
    break;

  case TRIMESH_UNIFORM_OPT:
    io_info.use_triangle_mesh = true;
    io_info.quad_tri_method = UNIFORM_TRI;
    break;

  case TRI4_CENTROID_OPT:
    io_info.tri4_position_method = TRI4_CENTROID;
    io_info.is_tri4_position_method_set = true;
    break;

  case QEI_INTERPOLATE_SCALAR_OPT:
    io_info.quad_edge_intersection_method = QEI_INTERPOLATE_SCALAR;
    io_info.is_qei_method_set = true;
    break;

  case QEI_INTERPOLATE_COORD_OPT:
    io_info.quad_edge_intersection_method = QEI_INTERPOLATE_COORD;
    io_info.is_qei_method_set = true;
    break;

  case QEI_AVERAGE_OPT:
    io_info.quad_edge_intersection_method = QEI_AVERAGE;
    io_info.is_qei_method_set = true;
    break;

  case COLOR_VERT_OPT:
    io_info.flag_color_vert = true;
    break;

  case OFF_OPT:
    io_info.flag_output_off = true;
    io_info.is_file_format_set = true;
    break;

  case PLY_OPT:
    io_info.flag_output_ply = true;
    io_info.is_file_format_set = true;
    break;

  case IV_OPT:
    io_info.flag_output_iv = true;
    io_info.is_file_format_set = true;
    break;

  case OUTPUT_FILENAME_OPT:
    iarg++;
    if (iarg >= argc) usage_error();
    io_info.output_filename = argv[iarg];
    break;

  case OUTPUT_FILENAME_PREFIX_OPT:
    iarg++;
    if (iarg >= argc) usage_error();
    io_info.output_filename_prefix = argv[iarg];
    break;

  case STDOUT_OPT:
    io_info.flag_use_stdout = true;
    break;

  case LABEL_WITH_ISOVALUE_OPT:
    io_info.label_with_isovalue = true;
    break;

  case NO_WRITE_OPT:
    io_info.flag_nowrite = true;
    break;

  case SILENT_OPT:
    io_info.flag_silent = true;
    break;

  case NO_WARN_OPT:
    io_info.flag_no_warn = true;
    break;

  case TIME_OPT:
    io_info.flag_report_time = true;
    break;

  case INFO_OPT:
    io_info.flag_report_info = true;
    break;

  case USAGE_OPT:
    usage(cout, 50);
    break;

  case ALL_OPTIONS_OPT:
    usage_all(cout, 50);
    break;

  case HELP_OPT:
    help();
    break;

  case HELP_ALL_OPT:
    help_all();
    break;

  case OUT_ISOV_OPT:
    iarg++;
    if (iarg >= argc) usage_error();
    io_info.report_isov_filename = argv[iarg];
    io_info.flag_report_all_isov = true;
    break;

  default:
    return(false);
  };

  return(true);
}


template <typename IO_INFO_TYPE>
void process_isovalues_and_input_filename
(const int argc, char **argv, const int iarg, IO_INFO_TYPE & io_info)
{
  // check for more parameter tokens
  for (int j = iarg; j+1 < argc; j++) {
    OPTION_TYPE optA;
    if (options.GetOption(argv[j], optA) ||
        (argv[j][0] == '-' && !is_type<float>(argv[j]))) {
      // argv[iarg] is not an isovalue
      cerr << "Usage error. Illegal parameter: " << argv[iarg] << endl;
      cerr << endl;
      usage_error();
    }
  }

  if (iarg+2 > argc) {
    cerr << "Error.  Missing input isovalue or input file name." << endl;
    cerr << endl;
    usage_error();
  };

  // store isovalues
  for (int j = iarg; j+1 < argc; j++) {
    SCALAR_TYPE value;
    if (!IJK::string2val(argv[j], value)) {
      cerr << "Error. \"" << argv[j] << "\" is not a valid input isovalue." 
           << endl;
      usage_error();
    }

    io_info.isovalue_string.push_back(argv[j]);
    io_info.isovalue.push_back(value);
  }

  io_info.input_filename = argv[argc-1];
}


template <typename IO_INFO_TYPE>
void process_io_info(IO_INFO_TYPE & io_info)
{
  if (io_info.is_tri4_position_method_set) {
    if (!io_info.flag_tri4_quad) {
      io_info.use_triangle_mesh = true;
      io_info.quad_tri_method = TRI4_MAX_MIN_ANGLE;
      io_info.flag_tri4_quad = true;
    }
  }

  if (io_info.flag_subsample && io_info.subsample_resolution <= 1) {
    cerr << "Error.  Subsample resolution must be an integer greater than 1."
         << endl;
    exit(230);
  };

  if (io_info.output_filename != "" && io_info.flag_use_stdout) {
    cerr << "Error.  Can't use both -o and -stdout parameters."
         << endl;
    exit(230);
  };

  if (io_info.isovalue.size() > 1 && io_info.output_filename != "") {
    cerr << "Error.  Cannot specify output file when input contains"
         << endl;
    cerr << "  more than one isovalue." << endl;
    exit(233);
  }

  if (!io_info.is_file_format_set) {
    if (io_info.output_filename != "") {
      OUTPUT_FORMAT output_format =
        get_suffix_type
        (io_info.output_filename, output_file_type_list, OFF);

      io_info.SetOutputFormat(output_format);
      io_info.SetOutputFilename(output_format, io_info.output_filename);
    }
    else {
      io_info.flag_output_off = true;
    }
  }
  else {
    if (io_info.output_filename != "") {
      if (io_info.NumOutputFormats() > 1) {
        cerr << "Error. Cannot set output filename when more than one"
             << endl
             << "  output format is selected." << endl;
        exit(545);

      }

      io_info.SetOutputFilename(io_info.output_filename);
    }
  }


  if (io_info.flag_subsample && io_info.flag_supersample) {
    cerr << "Error.  Can't use both -subsample and -supersample parameters."
         << endl;
    exit(555);
  }
}


// parse command line
// control parameters, followed by one or more isovalues, 
// followed by input file name
void ISODUAL::parse_command_line(int argc, char **argv, IO_INFO & io_info)
{
  IJK::ERROR error;

  set_command_line_options();
  create_output_file_type_list(output_file_type_list);

  // Set defaults.
  io_info.quad_tri_method = MAX_MIN_ANGLE;

  if (argc == 1) { usage_error(); };

  int iarg = 1;
  while (iarg < argc && argv[iarg][0] == '-') {

    OPTION_TYPE optA;
    if (!options.GetOption(argv[iarg], optA)) {
      // Unknown parameter.  Possibly negative scalar value.
      break;
    }

    if (!process_option(optA, argc, argv, iarg, io_info)) {
      error.AddMessage
        ("Programming error. Option ", options.Option(optA).option_name, 
         " not implemented.");
      throw error;
    }

    iarg++;
  }

  // remaining parameters should be list of isovalues followed
  // by input file name

  process_isovalues_and_input_filename(argc, argv, iarg, io_info);
  process_io_info(io_info);
}


// Check input information/flags.
bool ISODUAL::check_input
(const IO_INFO & io_info, 
 const DUALISO_SCALAR_GRID_BASE & scalar_grid,
 IJK::ERROR & error)
{
  // Construct isosurface
  if (io_info.isovalue.size() > 1 && io_info.flag_use_stdout) {
    error.AddMessage
      ("Error.  Cannot use stdout for more than one isovalue.");
    return(false);
  }

  return(true);
}

// **************************************************
// READ NEARLY RAW RASTER DATA (nrrd) FILE
// **************************************************

void ISODUAL::read_nrrd_file
(const char * input_filename, DUALISO_SCALAR_GRID & scalar_grid, 
 NRRD_HEADER & nrrd_header, IO_TIME & io_time)
{
  ELAPSED_TIME wall_time;
  GRID_NRRD_IN<int, AXIS_SIZE_TYPE> nrrd_in;
  IJK::PROCEDURE_ERROR error("read_nrrd_file");

  nrrd_in.ReadScalarGrid(input_filename, scalar_grid, nrrd_header, error);
  if (nrrd_in.ReadFailed()) { throw error; }

  std::vector<COORD_TYPE> grid_spacing;
  nrrd_header.GetSpacing(grid_spacing);

  for (int d = 0; d < scalar_grid.Dimension(); d++) {
    scalar_grid.SetSpacing(d, grid_spacing[d]);
  };

  io_time.read_nrrd_time = wall_time.getElapsed();
}

void ISODUAL::read_nrrd_file
(const std::string input_filename, DUALISO_SCALAR_GRID & scalar_grid, 
 NRRD_HEADER & nrrd_header, IO_TIME & io_time)
{
  read_nrrd_file(input_filename.c_str(), scalar_grid, nrrd_header, io_time);
}


// **************************************************
// PATH_DELIMITER
// **************************************************

namespace {

#ifdef _WIN32
  const char PATH_DELIMITER = '\\';
#else
  const char PATH_DELIMITER = '/';
#endif
}


// **************************************************
// WRITE_DUAL_MESH
// **************************************************

// Write dual mesh with output format output_format.
void ISODUAL::write_dual_mesh
(const OUTPUT_INFO & output_info, const OUTPUT_FORMAT output_format,
 const vector<COORD_TYPE> & vertex_coord, const vector<VERTEX_INDEX> & plist)
{
  const int dimension = output_info.dimension;
  const int numv_per_simplex = output_info.num_vertices_per_isopoly;
  const bool flag_use_stdout = output_info.flag_use_stdout;
  ofstream output_file;
  string ofilename;
  PROCEDURE_ERROR error("write_dual_mesh");

  // Output vertices in counter-clockwise order around quadrilateral.
  const bool flag_reorder_quad_vertices = true;

  switch (output_format) {

  case OFF:
    if (!flag_use_stdout) {
      ofilename = output_info.output_off_filename;
      output_file.open(ofilename.c_str(), ios::out);
      if (dimension == 3) {
        ijkoutQuadOFF(output_file, dimension, vertex_coord, plist, 
                      flag_reorder_quad_vertices);
      }
      else {
        ijkoutOFF(output_file, dimension, numv_per_simplex,
                  vertex_coord, plist);
      }
      output_file.close();
    }
    else {
      if (dimension == 3) {
        ijkoutQuadOFF(dimension, vertex_coord, plist,
                      flag_reorder_quad_vertices);
      }
      else {
        ijkoutOFF(dimension, numv_per_simplex, vertex_coord, plist);
      }
    };
    break;

  case PLY:
    if (dimension == 3) {
      if (!flag_use_stdout) {
        ofilename = output_info.output_ply_filename;
        output_file.open(ofilename.c_str(), ios::out);
        ijkoutQuadPLY(output_file, dimension, vertex_coord, plist, 
                      flag_reorder_quad_vertices);
        output_file.close();
      }
      else {
        ijkoutQuadPLY(cout, dimension, vertex_coord, plist, 
                      flag_reorder_quad_vertices);
      }
    }
    else throw error("Illegal dimension. PLY format is only for dimension 3.");
    break;

  case IV:
    if (dimension == 3) {
      if (!flag_use_stdout) {
        ofilename = output_info.output_iv_filename;
        output_file.open(ofilename.c_str(), ios::out);
        ijkoutIV(output_file, dimension, vertex_coord, plist);
        output_file.close();
      }
      else {
        ijkoutIV(dimension, vertex_coord, plist);
      }
    }
    else throw error
      ("Illegal dimension. OpenInventor format is only for dimension 3.");
    break;

  default:
    throw error("Illegal output format.");
    break;
  }

  if (!flag_use_stdout && !output_info.flag_silent && ofilename != "")
    cout << "Wrote output to file: " << ofilename << endl;
}


// Write dual mesh.
void ISODUAL::write_dual_mesh
(const OUTPUT_INFO & output_info,
 const vector<COORD_TYPE> & vertex_coord, const vector<VERTEX_INDEX> & plist)
{
  IJK::PROCEDURE_ERROR error("write_dual_mesh");

  if (output_info.flag_output_off) {
    if (output_info.output_off_filename != "") {
      write_dual_mesh(output_info, OFF, vertex_coord, plist);
    }
    else {
      error.AddMessage("Programming error. Geomview OFF file name not set.");
      throw error;
    }
  }

  if (output_info.flag_output_ply) {
    if (output_info.output_ply_filename != "") {
      write_dual_mesh(output_info, PLY, vertex_coord, plist);
    }
    else {
      error.AddMessage("Programming error. PLY file name not set.");
      throw error;
    }
  }

  if (output_info.flag_output_iv) {
    if (output_info.output_iv_filename != "") {
      write_dual_mesh(output_info, IV, vertex_coord, plist);
    }
    else {
      error.AddMessage
        ("Programming error. OpenInventor .iv file name not set.");
      throw error;
    }
  }
}


// Write dual mesh and record output time.
void ISODUAL::write_dual_mesh
(const OUTPUT_INFO & output_info,
 const vector<COORD_TYPE> & vertex_coord, const vector<VERTEX_INDEX> & plist,
 IO_TIME & io_time)
{
  ELAPSED_TIME wall_time;

  write_dual_mesh(output_info, vertex_coord, plist);

  io_time.write_time += wall_time.getElapsed();
}


// Write dual mesh and color facets with output format output_format.
void ISODUAL::write_dual_mesh_color
(const OUTPUT_INFO & output_info, const OUTPUT_FORMAT output_format,
 const vector<COORD_TYPE> & vertex_coord, const vector<VERTEX_INDEX> & plist,
 const COLOR_TYPE * front_color, const COLOR_TYPE * back_color)
{
  const int dimension = output_info.dimension;
  const int numv_per_simplex = output_info.num_vertices_per_isopoly;
  const bool flag_use_stdout = output_info.flag_use_stdout;
  string ofilename;
  ofstream output_file;
  PROCEDURE_ERROR error("write_dual_mesh_color");

  switch (output_format) {

  case OFF:
    if (!flag_use_stdout) {
      ofilename = output_info.output_off_filename;
      output_file.open(ofilename.c_str(), ios::out);
      ijkoutColorFacesOFF(output_file, dimension, numv_per_simplex,
                          vertex_coord, plist, front_color, back_color);
      output_file.close();
    }
    else {
      ijkoutColorFacesOFF(std::cout, dimension, numv_per_simplex,
                          vertex_coord, plist, front_color, back_color);
    };
    break;

  case IV:
    if (dimension == 3) {
      if (!flag_use_stdout) {
        ofilename = output_info.output_iv_filename;
        output_file.open(ofilename.c_str(), ios::out);
        ijkoutIV(output_file, dimension, vertex_coord, plist);
        output_file.close();
      }
      else {
        ijkoutOFF(dimension, vertex_coord, plist);
      }
    }
    else throw error
      ("Illegal dimension. OpenInventor format is only for dimension 3.");
    break;

  default:
    throw error("Illegal output format.");
    break;
  }

  if (!flag_use_stdout && !output_info.flag_silent && ofilename != "")
    cout << "Wrote output to file: " << ofilename << endl;
}

void ISODUAL::write_dual_mesh_color
(const OUTPUT_INFO & output_info, 
 const vector<COORD_TYPE> & vertex_coord, const vector<VERTEX_INDEX> & plist,
 const COLOR_TYPE * front_color, const COLOR_TYPE * back_color)
{
  IJK::PROCEDURE_ERROR error("write_dual_mesh_color");

  if (output_info.flag_output_off) {
    if (output_info.output_off_filename != "") {
      write_dual_mesh_color
        (output_info, OFF, vertex_coord, plist, front_color, back_color);
    }
    else {
      error.AddMessage("Programming error. Geomview OFF file name not set.");
      throw error;
    }
  }

  if (output_info.flag_output_ply) {
    if (output_info.output_ply_filename != "") {
      write_dual_mesh_color
        (output_info, PLY, vertex_coord, plist, front_color, back_color);
    }
    else {
      error.AddMessage("Programming error. PLY file name not set.");
      throw error;
    }
  }

  if (output_info.flag_output_iv) {
    if (output_info.output_iv_filename != "") {
      write_dual_mesh_color
        (output_info, IV, vertex_coord, plist, front_color, back_color);
    }
    else {
      error.AddMessage
        ("Programming error. OpenInventor .iv file name not set.");
      throw error;
    }
  }
}

void ISODUAL::write_dual_mesh_color
(const OUTPUT_INFO & output_info,
 const vector<COORD_TYPE> & vertex_coord, const vector<VERTEX_INDEX> & plist,
 const COLOR_TYPE * front_color, const COLOR_TYPE * back_color,
 IO_TIME & io_time)
{
  ELAPSED_TIME wall_time;

  write_dual_mesh_color
    (output_info, vertex_coord, plist, front_color, back_color);

  io_time.write_time += wall_time.getElapsed();
}


// Write dual isosurface triangular mesh.
void ISODUAL::write_dual_tri_mesh
(const OUTPUT_INFO & output_info, const OUTPUT_FORMAT output_format,
 const std::vector<COORD_TYPE> & vertex_coord,
 const std::vector<VERTEX_INDEX> & tri_vert)
{
  const int NUMV_PER_QUAD = 4;
  const int NUMV_PER_TRI = 3;
  const int dimension = output_info.dimension;
  const bool flag_use_stdout = output_info.flag_use_stdout;
  ofstream output_file;
  string ofilename;
  PROCEDURE_ERROR error("write_dual_tri_mesh");

  if (dimension != 3) {
    error.AddMessage("Programming error.  Illegal dimension ", dimension, ".");
    error.AddMessage("   Routine only allowed for dimension 3.");
    throw error;
  }

  switch (output_format) {

  case OFF:
    if (!flag_use_stdout) {
      ofilename = output_info.output_off_filename;
      output_file.open(ofilename.c_str(), ios::out);
      ijkoutOFF(output_file, dimension, NUMV_PER_TRI,
                vertex_coord, tri_vert);
      output_file.close();
    }
    else {
      ijkoutOFF(dimension, NUMV_PER_TRI, vertex_coord, tri_vert);
    };
    break;

  case PLY:
    if (dimension == 3) {
      if (!flag_use_stdout) {
        ofilename = output_info.output_ply_filename;
        output_file.open(ofilename.c_str(), ios::out);
        ijkoutPLY(output_file, dimension, NUMV_PER_TRI, 
                  vertex_coord, tri_vert);
        output_file.close();
      }
      else {
        ijkoutPLY(cout, dimension, NUMV_PER_TRI, 
                  vertex_coord, tri_vert);
      }
    }
    else throw error("Illegal dimension. PLY format is only for dimension 3.");
    break;

  case IV:
    ijkoutIV(dimension, vertex_coord, tri_vert);
    break;

  default:
    throw error("Illegal output format.");
    break;
  }

  if (!flag_use_stdout && !output_info.flag_silent && ofilename != "")
    cout << "Wrote output to file: " << ofilename << endl;
}

/// Write dual isosurface triangular mesh.
/// @param output_info Output information.
/// @param vertex_coord List of vertex coordinates.
/// @param tri_vert[] List of triangle vertices.
///        tri_vert[3*i+k] is k'th vertex of triangle i.
void ISODUAL::write_dual_tri_mesh
(const OUTPUT_INFO & output_info,
 const std::vector<COORD_TYPE> & vertex_coord,
 const std::vector<VERTEX_INDEX> & tri_vert)
{
  IJK::PROCEDURE_ERROR error("write_dual_tri_mesh");

  if (output_info.flag_output_off) {
    if (output_info.output_off_filename != "") {
      write_dual_tri_mesh(output_info, OFF, vertex_coord, tri_vert);
    }
    else {
      error.AddMessage("Programming error. Geomview OFF file name not set.");
      throw error;
    }
  }

  if (output_info.flag_output_ply) {
    if (output_info.output_ply_filename != "") {
      write_dual_tri_mesh(output_info, PLY, vertex_coord, tri_vert);
    }
    else {
      error.AddMessage("Programming error. PLY file name not set.");
      throw error;
    }
  }

  if (output_info.flag_output_iv) {
    if (output_info.output_iv_filename != "") {
      write_dual_tri_mesh(output_info, IV, vertex_coord, tri_vert);
    }
    else {
      error.AddMessage
        ("Programming error. OpenInventor .iv file name not set.");
      throw error;
    }
  }
}


void ISODUAL::write_dual_tri_mesh
(const OUTPUT_INFO & output_info,
 const vector<COORD_TYPE> & vertex_coord,
 const vector<VERTEX_INDEX> & tri_vert,
 IO_TIME & io_time)
{
  ELAPSED_TIME wall_time;

  write_dual_tri_mesh(output_info, vertex_coord, tri_vert);

  io_time.write_time += wall_time.getElapsed();
}

/// Write dual isosurface mesh of quad and triangles.
/// @param output_info Output information.
/// @param vertex_coord List of vertex coordinates.
void ISODUAL::write_dual_quad_tri_mesh
(const OUTPUT_INFO & output_info, const OUTPUT_FORMAT output_format,
 const std::vector<COORD_TYPE> & vertex_coord,
 const std::vector<VERTEX_INDEX> & quad_vert,
 const std::vector<VERTEX_INDEX> & tri_vert)
{
  const int NUMV_PER_QUAD = 4;
  const int NUMV_PER_TRI = 3;
  const int dimension = output_info.dimension;
  const bool flag_use_stdout = output_info.flag_use_stdout;
  std::vector<VERTEX_INDEX> quad_vert2;
  ofstream output_file;
  string ofilename;
  PROCEDURE_ERROR error("write_dual_quad_tri_mesh");

  if (dimension != 3) {
    error.AddMessage("Programming error.  Illegal dimension ", dimension, ".");
    error.AddMessage("   Routine only allowed for dimension 3.");
    throw error;
  }


  quad_vert2.resize(quad_vert.size());
  std::copy(quad_vert.begin(), quad_vert.end(), quad_vert2.begin());

  IJK::reorder_quad_vertices(quad_vert2);

  switch (output_format) {

    case OFF:
      if (!flag_use_stdout) {
        ofilename = output_info.output_off_filename;
        output_file.open(ofilename.c_str(), ios::out);
        ijkoutOFF(output_file, dimension, vertex_coord, 
                  quad_vert2, NUMV_PER_QUAD, tri_vert, NUMV_PER_TRI);
        output_file.close();
      }
      else {
        throw error("Output stdout not implemented.");
      };
      break;

    case PLY:
      if (dimension == 3) {
        if (!flag_use_stdout) {
          ofilename = output_info.output_ply_filename;
          output_file.open(ofilename.c_str(), ios::out);
          ijkoutPLY(output_file, dimension, vertex_coord,
                    quad_vert2, NUMV_PER_QUAD, tri_vert, NUMV_PER_TRI);
          output_file.close();
        }
        else {
          ijkoutPLY(cout, dimension, vertex_coord,
                    quad_vert2, NUMV_PER_QUAD, tri_vert, NUMV_PER_TRI);
        }
      }
      else throw error
        ("Illegal dimension. PLY format is only for dimension 3.");
    break;

    default:
      throw error("Illegal output format.");
      break;
  }

  if (!flag_use_stdout && !output_info.flag_silent && ofilename != "")
    cout << "Wrote output to file: " << ofilename << endl;
}

/// Write dual isosurface mesh of quad and triangles.
/// @param output_info Output information.
/// @param vertex_coord List of vertex coordinates.
void ISODUAL::write_dual_quad_tri_mesh
(const OUTPUT_INFO & output_info,
 const std::vector<COORD_TYPE> & vertex_coord,
 const std::vector<VERTEX_INDEX> & quad_vert,
 const std::vector<VERTEX_INDEX> & tri_vert)
{
  IJK::PROCEDURE_ERROR error("write_dual_quad_tri_mesh");

  if (output_info.flag_output_off) {
    if (output_info.output_off_filename != "") {
      write_dual_quad_tri_mesh
        (output_info, OFF, vertex_coord, quad_vert, tri_vert);
    }
    else {
      error.AddMessage("Programming error. Geomview OFF file name not set.");
      throw error;
    }
  }

  if (output_info.flag_output_ply) {
    if (output_info.output_ply_filename != "") {
      write_dual_quad_tri_mesh
        (output_info, PLY, vertex_coord, quad_vert, tri_vert);
    }
    else {
      error.AddMessage("Programming error. PLY file name not set.");
      throw error;
    }
  }

  if (output_info.flag_output_iv) {
    cerr << "Warning: Writing quad/tri mesh not implemented"
         << endl
         << "  for OpenInventor .iv format." 
         << endl;
    cerr << "Skipping output of OpenInventor .iv format." << endl;
  }
}


void ISODUAL::write_dual_quad_tri_mesh
(const OUTPUT_INFO & output_info,
 const std::vector<COORD_TYPE> & vertex_coord,
 const std::vector<VERTEX_INDEX> & quad_vert,
 const std::vector<VERTEX_INDEX> & tri_vert,
 IO_TIME & io_time)
{
  ELAPSED_TIME wall_time;

  write_dual_quad_tri_mesh(output_info, vertex_coord, quad_vert, tri_vert);

  io_time.write_time += wall_time.getElapsed();
}


// Write dual isosurface triangular mesh, color vertices
// @param output_info Output information.
// @param vertex_coord List of vertex coordinates.
// @param tri_vert[] List of triangle vertices.
//        tri_vert[3*i+k] is k'th vertex of triangle i.
void ISODUAL::write_dual_tri_mesh_color_vertices
(const OUTPUT_INFO & output_info, const OUTPUT_FORMAT output_format,
 const std::vector<COORD_TYPE> & vertex_coord,
 const std::vector<VERTEX_INDEX> & tri_vert,
 const COLOR_TYPE * front_color, const COLOR_TYPE * back_color)
{
  const int NUMV_PER_QUAD = 4;
  const int NUMV_PER_TRI = 3;
  const int dimension = output_info.dimension;
  const bool flag_use_stdout = output_info.flag_use_stdout;
  ofstream output_file;
  string ofilename;
  PROCEDURE_ERROR error("write_dual_tri_mesh");

  if (dimension != 3) {
    error.AddMessage("Programming error.  Illegal dimension ", dimension, ".");
    error.AddMessage("   Routine only allowed for dimension 3.");
    throw error;
  }

  switch (output_format) {

    case OFF:
      if (!flag_use_stdout) {
        ofilename = output_info.output_off_filename;
        output_file.open(ofilename.c_str(), ios::out);
        ijkoutColorVertOFF(output_file, dimension, NUMV_PER_TRI,
                           vertex_coord, tri_vert, front_color, back_color);
        output_file.close();
      }
      else {
        ijkoutColorVertOFF(std::cout, dimension, NUMV_PER_TRI,
                           vertex_coord, tri_vert, front_color, back_color);
      };
      break;

    default:
      throw error("Illegal output format.");
      break;
  }

  if (!flag_use_stdout && !output_info.flag_silent && ofilename != "")
    cout << "Wrote output to file: " << ofilename << endl;
}


// Write dual isosurface triangular mesh, color vertices
// @param output_info Output information.
// @param vertex_coord List of vertex coordinates.
// @param tri_vert[] List of triangle vertices.
//        tri_vert[3*i+k] is k'th vertex of triangle i.
void ISODUAL::write_dual_tri_mesh_color_vertices
(const OUTPUT_INFO & output_info,
 const std::vector<COORD_TYPE> & vertex_coord,
 const std::vector<VERTEX_INDEX> & tri_vert,
 const COLOR_TYPE * front_color, const COLOR_TYPE * back_color)
{
  IJK::PROCEDURE_ERROR error("write_dual_tri_mesh_color_vertices");

  if (output_info.flag_output_off) {
    if (output_info.output_off_filename != "") {
      write_dual_tri_mesh_color_vertices
        (output_info, OFF, vertex_coord, tri_vert, front_color, back_color);
    }
    else {
      error.AddMessage("Programming error. Geomview OFF file name not set.");
      throw error;
    }
  }

  if (output_info.flag_output_ply) {
    cerr << "Warning: Writing tri mesh with colored vertices not"
         << endl
         << "  implemented for .ply format." 
         << endl;
    cerr << "Skipping output of .ply format." << endl;
  }

  if (output_info.flag_output_iv) {
    cerr << "Warning: Writing tri mesh with colored vertices not"
         << endl
         << "  implemented for OpenInventor .iv format." 
         << endl;
    cerr << "Skipping output of OpenInventor .iv format." << endl;
  }
}

void ISODUAL::write_dual_tri_mesh_color_vertices
(const OUTPUT_INFO & output_info,
 const std::vector<COORD_TYPE> & vertex_coord,
 const std::vector<VERTEX_INDEX> & tri_vert,
 const COLOR_TYPE * front_color, const COLOR_TYPE * back_color,
 IO_TIME & io_time)
{
  ELAPSED_TIME wall_time;

  write_dual_tri_mesh_color_vertices
    (output_info, vertex_coord, tri_vert, front_color, back_color);

  io_time.write_time += wall_time.getElapsed();
}


/// Write dual isosurface mesh of quad and triangles.  Color vertices.
/// @param output_info Output information.
/// @param vertex_coord List of vertex coordinates.
void ISODUAL::write_dual_quad_tri_mesh_color_vertices
(const OUTPUT_INFO & output_info, const OUTPUT_FORMAT output_format,
 const std::vector<COORD_TYPE> & vertex_coord,
 const std::vector<VERTEX_INDEX> & quad_vert,
 const std::vector<VERTEX_INDEX> & tri_vert,
 const COLOR_TYPE * front_color, const COLOR_TYPE * back_color)
{
  const int NUMV_PER_QUAD = 4;
  const int NUMV_PER_TRI = 3;
  const int dimension = output_info.dimension;
  const bool flag_use_stdout = output_info.flag_use_stdout;
  std::vector<VERTEX_INDEX> quad_vert2;
  ofstream output_file;
  string ofilename;
  PROCEDURE_ERROR error("write_dual_quad_tri_mesh_color_vertices");

  if (dimension != 3) {
    error.AddMessage("Programming error.  Illegal dimension ", dimension, ".");
    error.AddMessage("   Routine only allowed for dimension 3.");
    throw error;
  }


  quad_vert2.resize(quad_vert.size());
  std::copy(quad_vert.begin(), quad_vert.end(), quad_vert2.begin());

  IJK::reorder_quad_vertices(quad_vert2);

  switch (output_format) {

    case OFF:
      if (!flag_use_stdout) {
        ofilename = output_info.output_off_filename;
        output_file.open(ofilename.c_str(), ios::out);
        ijkoutColorVertOFF
          (output_file, dimension, vertex_coord, 
           quad_vert2, NUMV_PER_QUAD, tri_vert, NUMV_PER_TRI,
           front_color, back_color);
        output_file.close();
      }
      else {
        throw error("Output stdout not implemented.");
      };
      break;

    default:
      throw error("Illegal output format.");
      break;
  }

  if (!flag_use_stdout && !output_info.flag_silent && ofilename != "")
    cout << "Wrote output to file: " << ofilename << endl;
}


/// Write dual isosurface mesh of quad and triangles.  Color vertices.
/// @param output_info Output information.
/// @param vertex_coord List of vertex coordinates.
void ISODUAL::write_dual_quad_tri_mesh_color_vertices
(const OUTPUT_INFO & output_info,
 const std::vector<COORD_TYPE> & vertex_coord,
 const std::vector<VERTEX_INDEX> & quad_vert,
 const std::vector<VERTEX_INDEX> & tri_vert,
 const COLOR_TYPE * front_color, const COLOR_TYPE * back_color)
{
  IJK::PROCEDURE_ERROR error("write_dual_quad_tri_mesh_color_vertices");

  if (output_info.flag_output_off) {
    if (output_info.output_off_filename != "") {
      write_dual_quad_tri_mesh_color_vertices
        (output_info, OFF, vertex_coord, quad_vert, tri_vert,
         front_color, back_color);
    }
    else {
      error.AddMessage("Programming error. Geomview OFF file name not set.");
      throw error;
    }
  }

  if (output_info.flag_output_ply) {
    cerr << "Warning: Writing quad/tri mesh with colored vertices not"
         << endl
         << "  implemented for .ply format." 
         << endl;
    cerr << "Skipping output of .ply format." << endl;
  }

  if (output_info.flag_output_iv) {
    cerr << "Warning: Writing quad/tri mesh with colored vertices not"
         << endl
         << "  implemented for OpenInventor .iv format." 
         << endl;
    cerr << "Skipping output of OpenInventor .iv format." << endl;
  }
}


void ISODUAL::write_dual_quad_tri_mesh_color_vertices
(const OUTPUT_INFO & output_info,
 const std::vector<COORD_TYPE> & vertex_coord,
 const std::vector<VERTEX_INDEX> & quad_vert,
 const std::vector<VERTEX_INDEX> & tri_vert,
 const COLOR_TYPE * front_color, const COLOR_TYPE * back_color,
 IO_TIME & io_time)
{
  ELAPSED_TIME wall_time;

  write_dual_quad_tri_mesh_color_vertices
    (output_info, vertex_coord, quad_vert, tri_vert, front_color, back_color);

  io_time.write_time += wall_time.getElapsed();
}


// **************************************************
// RESCALE ROUTINES
// **************************************************

namespace {

  void grow_coord(const int scale, vector<COORD_TYPE> & vertex_coord)
  {
    for (unsigned int i = 0; i < vertex_coord.size(); i++) {
      vertex_coord[i] = scale * vertex_coord[i];
    };
  }

  void shrink_coord(const int scale, vector<COORD_TYPE> & vertex_coord)
  {
    for (unsigned int i = 0; i < vertex_coord.size(); i++) {
      vertex_coord[i] = vertex_coord[i]/scale;
    };
  }

  bool unit_spacing(const int dimension, const COORD_TYPE * spacing)
  {
    for (unsigned int d = 0; d < dimension; d++) {
      if (!AIR_EXISTS(spacing[d])) { return(true); }
      else if (spacing[d] != 1.0) { return(false); };
    }

    return(true);
  }

  bool unit_spacing(const std::vector<COORD_TYPE> & spacing)
  // return true if spacing not defined or spacing along all axes equals 1.0
  {
    return(spacing.size(), IJK::vector2pointer(spacing));
  }

}

void ISODUAL::rescale_vertex_coord
(const int dimension, const COORD_TYPE * grid_spacing,
 std::vector<COORD_TYPE> & vertex_coord)
{
  if (unit_spacing(dimension, grid_spacing)) { return; }

  if (vertex_coord.size() == 0) { return; };

  const VERTEX_INDEX numv = vertex_coord.size()/dimension;
  for (int iv = 0; iv < numv; iv++) {
    for (int d = 0; d < dimension; d++) {
      vertex_coord[iv*dimension+d] *= grid_spacing[d];
    }
  }
}

void ISODUAL::rescale_vertex_coord(const std::vector<COORD_TYPE> & grid_spacing,
                                   std::vector<COORD_TYPE> & vertex_coord)
{
  const int dimension = grid_spacing.size();
  PROCEDURE_ERROR error("rescale_vertex_coord");

  if (grid_spacing.size() < 1) {
    error.AddMessage("Illegal size ", grid_spacing.size(), 
                     " of array grid spacing.");
    error.AddMessage("Size must equal vertex dimension.");
    throw error;
  }

  rescale_vertex_coord(dimension, vector2pointer(grid_spacing),
                       vertex_coord);
}


/// Rescale subsampled/supersampled vertex coordinates.
void ISODUAL::rescale_vertex_coord
(const int grow_factor, const int shrink_factor, COORD_ARRAY & vertex_coord)
{
  PROCEDURE_ERROR error("rescale_vertex_coord");

  if (grow_factor <= 0) {
    error.AddMessage("Illegal grow factor ", grow_factor, ".");
    error.AddMessage("  Grow factor must be a positive integer");
  }

  if (shrink_factor <= 0) {
    error.AddMessage("Illegal shrink factor ", shrink_factor, ".");
    error.AddMessage("  Shrink factor must be a positive integer");
  }

  if (vertex_coord.size() == 0) { return; };

  if (grow_factor != 1) 
    { grow_coord(grow_factor, vertex_coord); };

  if (shrink_factor != 1) 
    { shrink_coord(shrink_factor, vertex_coord); };
}

/// Rescale subsampled/supersampled vertex coordinates.
/// Also rescale to reflect grid spacing.
void ISODUAL::rescale_vertex_coord
(const int grow_factor, const int shrink_factor,
 const COORD_ARRAY & grid_spacing, COORD_ARRAY & vertex_coord)
{
  rescale_vertex_coord(grow_factor, shrink_factor, vertex_coord);
  rescale_vertex_coord(grid_spacing, vertex_coord);
}

// **************************************************
// REPORT SCALAR FIELD OR ISOSURFACE INFORMATION
// **************************************************

void ISODUAL::report_num_cubes
(const DUALISO_GRID & full_scalar_grid, const IO_INFO & io_info, 
 const DUALISO_DATA & dualiso_data)
{
  report_num_cubes(full_scalar_grid, io_info, dualiso_data.ScalarGrid());
}


void ISODUAL::report_num_cubes
(const DUALISO_GRID & full_scalar_grid, const IO_INFO & io_info, 
 const DUALISO_GRID & dualiso_data_grid)
{
  const int num_grid_cubes = full_scalar_grid.ComputeNumCubes();
  const int num_cubes_in_dualiso_data = dualiso_data_grid.ComputeNumCubes();

  if (!io_info.flag_use_stdout && !io_info.flag_silent) {

    if (io_info.flag_subsample) {
      // subsampled grid
      cout << num_grid_cubes << " grid cubes.  "
           << num_cubes_in_dualiso_data << " subsampled grid cubes." << endl;
    }
    else if (io_info.flag_supersample) {
      // supersample grid
      cout << num_grid_cubes << " grid cubes.  "
           << num_cubes_in_dualiso_data << " supersampled grid cubes." << endl;
    }
    else {
      // use full_scalar_grid
      cout << num_grid_cubes << " grid cubes." << endl;
    }
  }

}


void ISODUAL::warn_non_manifold(const IO_INFO & io_info)
{
  const char * mesh_str = "mesh";

  if (io_info.flag_use_stdout || io_info.flag_silent ||
      io_info.flag_no_warn) { return; }

  if (io_info.isovalue.size() > 1) { mesh_str = "meshes"; }

  if (!io_info.allow_multiple_iso_vertices ||
      !io_info.flag_split_non_manifold) {

    cout << "***Warning: Isosurface " << mesh_str
         << " may have non-manifold vertices or edges."   << endl;
    cout << endl;
  }
}


// **************************************************
// REPORT TIMING INFORMATION
// **************************************************

void ISODUAL::report_dualiso_time
(const IO_INFO & io_info, const DUALISO_TIME & dualiso_time, 
 const char * mesh_type_string)
{
  cout << "CPU time to run isodual: " 
       << dualiso_time.total << " seconds." << endl;
  cout << "    Time to extract " << mesh_type_string << " triangles: "
       << dualiso_time.extract << " seconds." << endl;
  cout << "    Time to merge identical "
       << mesh_type_string << " vertices: " 
       << dualiso_time.merge << " seconds." << endl;
  cout << "    Time to position "
       << mesh_type_string << " vertices: "
       << dualiso_time.position << " seconds." << endl;
  if (io_info.flag_dual_collapse) {
    cout << "    Time to merge " << mesh_type_string << " vertices: "
         << dualiso_time.collapse << " seconds." << endl;
  }
}


void ISODUAL::report_time
(const IO_INFO & io_info, const IO_TIME & io_time, 
 const DUALISO_TIME & dualiso_time, const double total_elapsed_time)
{
  const char * ISOSURFACE_STRING = "isosurface";
  const char * INTERVAL_VOLUME_STRING = "interval volume";
  const char * mesh_type_string = NULL;
  
  mesh_type_string = ISOSURFACE_STRING;

  cout << "Time to read file " << io_info.input_filename << ": "
       << io_time.read_nrrd_time << " seconds." << endl;

  report_dualiso_time(io_info, dualiso_time, mesh_type_string);
  if (!io_info.flag_nowrite) {
    cout << "Time to write "
         << mesh_type_string << ": " 
         << io_time.write_time << " seconds." << endl;
  };
  cout << "Total elapsed time: " << total_elapsed_time
       << " seconds." << endl;
}

// **************************************************
// USAGE/HELP MESSAGES
// **************************************************

// local namespace
namespace {

  void usage_msg(std::ostream & out)
  {
    out << "Usage: isodual [OPTIONS] {isovalue1 isovalue2 ...} {input filename}" << endl;
  }

  void print_options_title(std::ostream & out, const OPTION_GROUP group)
  {
    switch(group) {
      
    case REGULAR_OPTG:
      out << "OPTIONS:" << endl;
      break;

    case EXTENDED_OPTG:
      out << "MORE OPTIONS:" << endl;
      break;

    case QDUAL_OPTG:
      out << "QDUAL OPTIONS:" << endl;
      break;

    case TESTING_OPTG:
      out << "TESTING OPTIONS:" << endl;
      break;

    default:
      out << "OTHER OPTIONS:" << endl;
      break;
    }
  }

  void options_msg(std::ostream & out, const OPTION_GROUP group)
  {
    print_options_title(out, group);
    options.PrintUsageOptions(out, group);
  };

  void help_msg()
  {
    usage_msg(cout);
    cout << endl;
    cout << "isodual - Dual contouring isosurface generation algorithm." 
         << endl;
  }

  void print_help_options(const OPTION_GROUP group)
  {
    print_options_title(cout, group);

    for (int i = 0; i < options.list.size(); i++) {
      if (options.list[i].Group() == group)
        { options.PrintHelpMessage(cout, i); }
    }
  }
}

void ISODUAL::usage(std::ostream & out, const int return_code)
{
  usage_msg(out);
  options_msg(out, REGULAR_OPTG);
  exit(return_code);
}

void ISODUAL::usage_error()
{
  usage(cerr, 10);
}

void ISODUAL::usage_all(std::ostream & out, const int return_code)
{
  usage_msg(out);
  options_msg(out, REGULAR_OPTG);
  options_msg(out, EXTENDED_OPTG);
  options_msg(out, QDUAL_OPTG);
  options_msg(out, TESTING_OPTG);
  exit(return_code);
}

void ISODUAL::help_all()
{
  help_msg();
  cout << endl;

  print_help_options(REGULAR_OPTG);
  cout << endl;
  print_help_options(EXTENDED_OPTG);
  cout << endl;
  print_help_options(QDUAL_OPTG);
  cout << endl;
  print_help_options(TESTING_OPTG);
  cout << endl;

  exit(20);
}

void ISODUAL::help()
{
  help_msg();
  cout << endl;
  print_help_options(REGULAR_OPTG);
  exit(20);
}


// **************************************************
// CLASS IO_INFO
// **************************************************

/// IO information
void ISODUAL::IO_INFO::Init()
{
  isovalue.clear();
  isovalue_string.clear();
  input_filename .clear();
  isotable_directory = "";
  label_with_isovalue = false;
  flag_output_off = false;
  flag_output_ply = false;
  flag_output_iv = false;
  are_output_filenames_set = false;
  flag_report_time = false;
  flag_report_info = false;
  flag_use_stdout = false;
  flag_nowrite = false;
  flag_silent = false;
  flag_no_warn = false;
  flag_subsample = false;
  flag_report_all_isov = false;
  subsample_resolution = 2;
  flag_supersample = false;
  supersample_resolution = 2;
  flag_color_alternating = false;  // color simplices in alternating cubes
  flag_color_vert = false;         // color isosurface boundary vertices
  region_length = 1;

  is_qei_method_set = false;
  is_tri4_position_method_set = false;
  is_file_format_set = false;
}


int ISODUAL::IO_INFO::NumOutputFormats() const
{
  int num_output_formats = 0;

  if (flag_output_off) { num_output_formats++; }
  if (flag_output_ply) { num_output_formats++; }
  if (flag_output_iv) { num_output_formats++; }

  return(num_output_formats);
}

void ISODUAL::IO_INFO::Set(const IO_INFO & io_info)
{
  *this = io_info;
}


void ISODUAL::IO_INFO::SetOutputFormat(const OUTPUT_FORMAT output_format)
{
  IJK::PROCEDURE_ERROR error("IO_INFO::SetOutputFormat");

  switch(output_format) {

  case OFF:
    flag_output_off = true;
    break;

  case PLY:
    flag_output_ply = true;
    break;

  case IV:
    flag_output_iv = true;
    break;

  default:
    error.AddMessage
      ("Programming error. Unable to set output format to ",
       get_suffix_string(output_format, output_file_type_list, "UNKNOWN"), ".");
    throw error;
  }
}


// Set output filename for output format flagged true.
void ISODUAL::IO_INFO::SetOutputFilename(const char * output_filename)
{
  IJK::PROCEDURE_ERROR error("IO_INFO::SetOutputFilename");

  int num_output_formats = 0;

  if (flag_output_off) {
    output_off_filename = output_filename;
    num_output_formats++;
    are_output_filenames_set = true;
  }

  if (flag_output_ply) {
    output_ply_filename = output_filename;
    num_output_formats++;
    are_output_filenames_set = true;
  }

  if (flag_output_iv) {
    output_iv_filename = output_filename;
    num_output_formats++;
    are_output_filenames_set = true;
  }

  if (num_output_formats > 1) {
    error.AddMessage
      ("Programming error.  More than one output format is set.");
    error.AddMessage
      ("  Cannot set same output filename for more than one output format.");
    throw error;
  }
}


// Set output filename for a given output format.
void ISODUAL::IO_INFO::SetOutputFilename
(const OUTPUT_FORMAT output_format, const char * output_filename)
{
  IJK::PROCEDURE_ERROR error("IO_INFO::SetOutputFilename");

  switch(output_format) {

  case OFF:
    output_off_filename = output_filename;
    break;

  case PLY:
    output_ply_filename = output_filename;
    break;

  case IV:
    output_iv_filename = output_filename;
    break;

  default:
    error.AddMessage
      ("Programming error.  Unknown file type ",
       get_suffix_string(output_format, output_file_type_list, "UNKNOWN"), 
       ".");
    throw error;
  }

  are_output_filenames_set = true;
}


void ISODUAL::IO_INFO::ConstructOutputFilenames(const int i)
{
  string prefix, suffix;
  string ofilename;

  // create output filename
  if (output_filename_prefix == "") {
    string fname = string(input_filename);

#ifndef _WIN32
    // remove path from file name
    split_string(fname, PATH_DELIMITER, prefix, suffix);
    if (suffix != "") { fname = suffix; }
#endif

    // construct output filename
    split_string(fname, '.', prefix, suffix);
    if (suffix == "nrrd" || suffix == "nhdr") { ofilename = prefix; }
    else { ofilename = string(input_filename); }
  }
  else {
    ofilename = output_filename_prefix;
  }

  if (label_with_isovalue || isovalue_string.size() > 1) {
    ofilename += string(".") + string("isov=") + isovalue_string[i];
  }

  output_off_filename = ofilename + ".off";
  output_ply_filename = ofilename + ".ply";
  output_iv_filename = ofilename + ".iv";
}



// **************************************************
// class OUTPUT_INFO
// **************************************************

void ISODUAL::OUTPUT_INFO::Init()
{
  dimension = 3;
  num_vertices_per_isopoly = 4;
  isovalue[0] = 0;
  isovalue[1] = 0;
  grow_factor = 1;
  shrink_factor = 1;
  grid_spacing.resize(3,1);
}


void ISODUAL::OUTPUT_INFO::SetDimension(const int d)
{
  IJK::PROCEDURE_ERROR error("OUTPUT_INFO::SetDimension");

  dimension = d;

  if (use_triangle_mesh) {
    num_vertices_per_isopoly = d+1;
  }
  else {
    IJK::int_power(2, (dimension-1), num_vertices_per_isopoly, error);
  }

}


namespace {

  void split_string(const string & s, const char c,
                    string & prefix, string & suffix)
  // split string at last occurrence of character c into prefix and suffix
  {
    string::size_type i = s.rfind(c);
    if (i == string::npos) {
      prefix = s;
      suffix = "";
    }
    else {
      if (i > 0) { prefix = s.substr(0,i); }
      else { prefix = ""; };

      if (i+1 < s.length()) { suffix = s.substr(i+1, s.length()-i-1); }
      else { suffix = ""; };
    }
  }

}


// **************************************************
// SET ROUTINES
// **************************************************

void ISODUAL::set_output_info
(const IO_INFO & io_info, 
 const int i, OUTPUT_INFO & output_info)
{
  output_info.Set(io_info);

  if (output_info.use_triangle_mesh) {
    output_info.num_vertices_per_isopoly = 3;
  }

  output_info.grow_factor = 1;
  if (io_info.flag_subsample) 
    { output_info.grow_factor = io_info.subsample_resolution; }

  output_info.shrink_factor = 1;
  if (io_info.flag_supersample) 
    { output_info.shrink_factor = io_info.supersample_resolution; }

  output_info.grid_spacing.clear();
  output_info.grid_spacing.resize(io_info.grid_spacing.size());
  for (unsigned int j = 0; j < io_info.grid_spacing.size(); j++)
    { output_info.grid_spacing[j] = io_info.grid_spacing[j]; }

  output_info.isovalue[0] = io_info.isovalue[i];
  if (i+1 < int(io_info.isovalue.size())) 
    { output_info.isovalue[1] = io_info.isovalue[i+1]; };

  if (!io_info.are_output_filenames_set) {
    output_info.ConstructOutputFilenames(i);
  }
}

void ISODUAL::set_color_alternating
(const DUALISO_GRID & grid, const vector<VERTEX_INDEX> & cube_list, 
 COLOR_TYPE * color)
{
  const int dimension = grid.Dimension();
  IJK::ARRAY<GRID_COORD_TYPE> coord(dimension);

  const COLOR_TYPE red[3] = { 1.0, 0.0, 0.0 };
  const COLOR_TYPE blue[3] = { 0.0, 0.0, 1.0 };
  const COLOR_TYPE green[3] = { 0.0, 1.0, 0.0 };

  VERTEX_INDEX icube = 0;
  int parity = 0;
  COLOR_TYPE * color_ptr = color;
  for (unsigned int i = 0; i < cube_list.size(); i++) {
    int new_cube = cube_list[i];
    if (icube != new_cube) {
      icube = new_cube;
      grid.ComputeCoord(icube, coord.Ptr());
      int sum = 0;
      for (int d = 0; d < dimension; d++) 
        { sum += coord[d]; }
      parity = sum%2;
    }

    if (parity == 0) 
      { std::copy(red, red+3, color_ptr); }
    else
      { std::copy(blue, blue+3, color_ptr); }

    // set opacity
    color_ptr[3] = 1.0;
    color_ptr += 4;
  }
  
}

