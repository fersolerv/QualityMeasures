file(GLOB THREAD_GRASP_SRC *.cpp)

# Declare the library
add_library(THREAD_GRASP STATIC ${THREAD_GRASP_SRC})

# Specify here the include directories exported by this library
target_include_directories(THREAD_GRASP PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
