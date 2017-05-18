PROJECT(ISODUAL)

#---------------------------------------------------------

CMAKE_MINIMUM_REQUIRED(VERSION 2.8)

SET(NRRD_LIBDIR "~/programs/ijk/lib")

#---------------------------------------------------------

IF (NOT CMAKE_BUILD_TYPE)
  SET (CMAKE_BUILD_TYPE Release CACHE STRING 
       "Default build type: Release" FORCE)
ENDIF (NOT CMAKE_BUILD_TYPE)

LINK_DIRECTORIES("${NRRD_LIBDIR}")
LINK_LIBRARIES(NrrdIO z)

# Include random library.
SET(CMAKE_CXX_FLAGS "-std=c++11")

ADD_EXECUTABLE(isodual isodual_main.cxx isodualIO.cxx isodual.cxx
                       ijkdual_datastruct.cxx ijkdualtable.cxx)


ADD_CUSTOM_TARGET(tar WORKING_DIRECTORY . COMMAND tar cvfh isodual.tar *.cxx *.h *.txx CMakeLists.txt isodual_doxygen.config)

ADD_CUSTOM_TARGET(doc COMMAND doxygen isodual_doxygen.config)
