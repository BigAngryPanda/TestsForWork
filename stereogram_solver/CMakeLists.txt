# project(TestTask)

# cmake_minimum_required(VERSION 3.16)

# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")

# set(OpenCV_DIR ${PROJECT_SOURCE_DIR}/opencv_build/)

# find_package(OpenCV REQUIRED)

# set(OpenCV_STATIC ON)

# set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
# set(CMAKE_BINARY_DIR ${CMAKE_SOURCE_DIR}/build)
# set(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR})
# set(OPENCV ${PROJECT_SOURCE_DIR}/opencv_build/install/include/)

#set(SRC main.cpp
#        ${OPENCV}/opencv2/viz.hpp
#)

# include_directories(${PROJECT_SOURCE_DIR})
# include_directories(${OPENCV})

# link_directories(${PROJECT_SOURCE_DIR}/opencv_build/install/lib)

# add_executable(main ${SRC})

# target_link_libraries(main ${PROJECT_SOURCE_DIR}/opencv_build/install/lib/libopencv_viz.a -static)

cmake_minimum_required(VERSION 3.1)
project(TestTask)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -lpthread")
set(OpenCV_DIR "/usr/share/OpenCV")

option(DEBUG "Enable debug output" OFF)
if (DEBUG)
  add_definitions(-DDEBUG)
endif()

find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(CMAKE_BINARY_DIR ${CMAKE_SOURCE_DIR}/build)

set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)

include_directories(${PROJECT_SOURCE_DIR})
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${PCL_INCLUDE_DIRS})

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(main main.cpp file_list.cpp render.cpp stereogram_solver.cpp log.cpp)
target_link_libraries(main ${OpenCV_LIBS} ${PCL_LIBRARIES} Threads::Threads)