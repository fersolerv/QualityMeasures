 
#PUGI_XML
find_path(KAUTHAM_INCLUDE_DIRS kauthamshell.h
	  PATHS /home/users/abiud.rojas/catkin_ws/src/Kautham.git/src/applications/console 
	        /usr/include
                /user/local/include)

if(KAUTHAM_INCLUDE_DIRS)
  set(KAUTHAM_INCLUDE_DIRS ${KAUTHAM_INCLUDE_DIRS} ${KAUTHAM_INCLUDE_DIRS}/../.. ${KAUTHAM_INCLUDE_DIRS}/../../util/libmt) 
  find_library (KAUTHAM_LIBRARIES
                NAMES kautham libkautham
                PATHS /home/users/abiud.rojas/catkin_ws/devel/lib
                      /usr/lib
                      /usr/local/lib
                      /user/local/lib)
  if(KAUTHAM_LIBRARIES)
      set(KAUTHAM_FOUND true)
  endif(KAUTHAM_LIBRARIES)
endif(KAUTHAM_INCLUDE_DIRS)

# Support the REQUIRED and QUIET arguments, and set PUGIXML_FOUND if found.
#include (FindPackageHandleStandardArgs)
#find_package_handle_standard_args (PugiXML REQUIRED_VARS
 #                                 PUGI_XML_LIBRARIES
  #                                PUGI_XML_INCLUDE_DIRS)

if (KAUTHAM_FOUND)
    #set (KAUTHAM_LIBRARIES ${KAUTHAM_LIBRARY})
    message (STATUS "Found KAUTHAM includes on: ${KAUTHAM_INCLUDE_DIRS}")
    message (STATUS "Found KAUTHAM libraries on: ${KAUTHAM_LIBRARIES}")
else (KAUTHAM_FOUND)
    message (STATUS "No KAUTHAM found")
endif(KAUTHAM_FOUND)

mark_as_advanced (KAUTHAM_LIBRARIES )