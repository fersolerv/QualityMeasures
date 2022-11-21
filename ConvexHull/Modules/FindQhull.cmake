# ###############################################################################
# # Find QHULL
# #
# # This sets the following variables:
# # QHULL_FOUND - True if QHULL was found.
# # QHULL_INCLUDE_DIRS - Directories containing the QHULL include files.
# # QHULL_LIBRARIES - Libraries needed to use QHULL.
# # QHULL_DEFINITIONS - Compiler flags for QHULL.
# # If QHULL_USE_STATIC is specified then look for static libraries ONLY else
# # look for shared ones

# set(QHULL_MAJOR_VERSION 6)

# if(QHULL_USE_STATIC)
#   set(QHULL_RELEASE_NAME qhullstatic)
#   set(QHULL_DEBUG_NAME qhullstatic_d)
# else(QHULL_USE_STATIC)
#   set(QHULL_RELEASE_NAME qhull qhull${QHULL_MAJOR_VERSION})
#   set(QHULL_DEBUG_NAME qhull_d qhull${QHULL_MAJOR_VERSION}_d qhull_d${QHULL_MAJOR_VERSION})
# endif(QHULL_USE_STATIC)

# find_file(QHULL_HEADER
#           NAMES libqhull/libqhull.h qhull.h
#           HINTS "${QHULL_ROOT}" "$ENV{QHULL_ROOT}" "${QHULL_INCLUDE_DIR}"
#           PATHS "$ENV{PROGRAMFILES}/QHull" "$ENV{PROGRAMW6432}/QHull"
#           PATH_SUFFIXES qhull src/libqhull libqhull include)

# set(QHULL_HEADER "${QHULL_HEADER}" CACHE INTERNAL "QHull header" FORCE )

# if(QHULL_HEADER)
#   get_filename_component(qhull_header ${QHULL_HEADER} NAME_WE)
#   if("${qhull_header}" STREQUAL "qhull")
#     set(HAVE_QHULL_2020 OFF)
#     get_filename_component(QHULL_INCLUDE_DIR ${QHULL_HEADER} PATH)
#   elseif("${qhull_header}" STREQUAL "libqhull")
#     set(HAVE_QHULL_2020 ON)
#     get_filename_component(QHULL_INCLUDE_DIR ${QHULL_HEADER} PATH)
#     get_filename_component(QHULL_INCLUDE_DIR ${QHULL_INCLUDE_DIR} PATH)
#   endif()
# else(QHULL_HEADER)
#   set(QHULL_INCLUDE_DIR "QHULL_INCLUDE_DIR-NOTFOUND")
# endif(QHULL_HEADER)

# set(QHULL_INCLUDE_DIR "${QHULL_INCLUDE_DIR}" CACHE PATH "QHull include dir." FORCE)

# find_library(QHULL_LIBRARY
#              NAMES ${QHULL_RELEASE_NAME}
#              HINTS "${QHULL_ROOT}" "$ENV{QHULL_ROOT}"
#              PATHS "$ENV{PROGRAMFILES}/QHull" "$ENV{PROGRAMW6432}/QHull"
#              PATH_SUFFIXES project build bin lib)

# find_library(QHULL_LIBRARY_DEBUG
#              NAMES ${QHULL_DEBUG_NAME} ${QHULL_RELEASE_NAME}
#              HINTS "${QHULL_ROOT}" "$ENV{QHULL_ROOT}"
#              PATHS "$ENV{PROGRAMFILES}/QHull" "$ENV{PROGRAMW6432}/QHull"
#              PATH_SUFFIXES project build bin lib)

# if(NOT QHULL_LIBRARY_DEBUG)
#   set(QHULL_LIBRARY_DEBUG ${QHULL_LIBRARY})
# endif(NOT QHULL_LIBRARY_DEBUG)

# set(QHULL_INCLUDE_DIRS ${QHULL_INCLUDE_DIR})
# set(QHULL_LIBRARIES optimized ${QHULL_LIBRARY} debug ${QHULL_LIBRARY_DEBUG})

# include(FindPackageHandleStandardArgs)
# find_package_handle_standard_args(Qhull DEFAULT_MSG QHULL_LIBRARY QHULL_INCLUDE_DIR)

# mark_as_advanced(QHULL_LIBRARY QHULL_LIBRARY_DEBUG QHULL_INCLUDE_DIR)

# if(QHULL_FOUND)
#   set(HAVE_QHULL ON)
#   if(NOT QHULL_USE_STATIC)
#     add_definitions("-Dqh_QHpointer")
#     if(MSVC)
#       add_definitions("-Dqh_QHpointer_dllimport")
#     endif(MSVC)
#   endif(NOT QHULL_USE_STATIC)
#   message(STATUS "QHULL found (include: ${QHULL_INCLUDE_DIRS}, lib: ${QHULL_LIBRARIES})")

# endif(QHULL_FOUND)

# Searches for QHULL includes and library files
#
# Sets the variables
#   QHULL_FOUND
#   QHULL_INCLUDE_DIR
#   QHULL_LIBRARIES

SET (QHULL_FOUND FALSE)

FIND_PATH (QHULL_INCLUDE_DIR libqhull_r/qhull_ra.h
	/usr/include
	/usr/local/include
	$ENV{HOME}/local/include
	$ENV{QHULL_PATH}/src
	$ENV{QHULL_PATH}/include
	$ENV{QHULL_INCLUDE_PATH}
	)
FIND_LIBRARY (QHULL_LIBRARY NAMES qhull_r	PATHS
	/usr/lib
	/usr/local/lib
	$ENV{HOME}/local/lib
	$ENV{QHULL_PATH}
	$ENV{QHULL_LIBRARY_PATH}
	)


IF (QHULL_INCLUDE_DIR AND QHULL_LIBRARY)
	SET (QHULL_FOUND TRUE)
ENDIF (QHULL_INCLUDE_DIR AND QHULL_LIBRARY)

SET(QHULL_LIBRARIES ${QHULL_LIBRARY})

IF (QHULL_FOUND)
   IF (NOT QHULL_FIND_QUIETLY)
      MESSAGE(STATUS "Found QHULL: ${QHULL_LIBRARY}")
   ENDIF (NOT QHULL_FIND_QUIETLY)
ELSE (QHULL_FOUND)
   IF (QHULL_FIND_REQUIRED)
      MESSAGE(FATAL_ERROR "Could not find QHULL")
   ENDIF (QHULL_FIND_REQUIRED)
ENDIF (QHULL_FOUND)

MARK_AS_ADVANCED (
	QHULL_INCLUDE_DIR
	QHULL_LIBRARIES
	QHULL_LIBRARY	
	)
