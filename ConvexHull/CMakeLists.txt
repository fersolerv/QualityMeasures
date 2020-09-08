PROJECT(TransferQualityMeasure)
cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(TransferQualityMeasure)

set(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/Modules/" ${CMAKE_MODULE_PATH})

set(CMAKE_BUILD_TYPE RelWithDebInfo)

set(CMAKE_CXX_STANDARD 14)

find_package(Eigen REQUIRED)
INCLUDE_DIRECTORIES(SYSTEM ${EIGEN_INCLUDE_DIRS})
add_definitions(-DEIGEN_USE_NEW_STDVECTOR
                 -DEIGEN_YES_I_KNOW_SPARSE_MODULE_IS_NOT_STABLE_YET)
                 
find_package( PugiXML )
if(PUGI_XML_FOUND)
   message( STATUS "PugiXML FOUND, using it")
   set(EXTRA_LIBS ${EXTRA_LIBS} ${PUGI_XML_LIBRARIES} PARENT_SCOPE)
   include_directories( ${PUGI_XML_INCLUDE_DIR})
else(PUGI_XML_FOUND)
   message(STATUS "PugiXML not found, using internal copy")
   add_subdirectory ( pugixml )
   include_directories( ${CMAKE_CURRENT_SOURCE_DIR}/pugixml )
set(PUGI_XML_LIBRARIES libpugixml)
endif(PUGI_XML_FOUND)

include_directories(${PUGI_XML_INCLUDE_DIR})
                 
find_package(Qhull)

find_package(PCL 1.8 REQUIRED)
if (NOT PCL_SHARED_LIB OR WIN32)
set(QHULL_USE_STATIC ON)
endif(NOT PCL_SHARED_LIB OR WIN32)

INCLUDE_DIRECTORIES( ${CMAKE_SOURCE_DIR}
                     ${CMAKE_SOURCE_DIR}/include)
                     
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

message(STATUS "PCL_LIBRARY_DIRS(${PCL_LABRARY_DIRS})")

SET(SRC src/main.cpp 
        src/ContactConeGenerator.cpp 
        src/ConvexHull.cpp
        src/Mtools.cpp
        src/MVBB.cpp
        src/Wrenches.cpp)
        #src/FilterObject.cpp)

add_subdirectory(THREAD_POOL)
        
add_executable(TransferQualityMeasure ${SRC})


target_link_libraries(TransferQualityMeasure ${QHULL_LIBRARIES}
                                             #${COIN_LIBRARY}
                                             ${PUGI_XML_LIBRARIES}
                                             ${PCL_LIBRARIES}
                                             ${PCL_IO_LIBRARIES}
                                             THREAD_POOL
                                             pugixml)
                                             
#add_executable(FilterObject ${SRC})                                             
#target_link_libraries(FilterObject  ${PCL_LIBRARIES}
#                                    ${PCL_IO_LIBRARIES})
