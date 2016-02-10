cmake_minimum_required(VERSION 2.8)

PROJECT(raycaster)
SET(VTK_DIR /home/eewing/VTK-Builds/Standard/lib/cmake/vtk-6.1)

SET(CMAKE_BUILD_TYPE Debug)

find_package( Threads )
find_package(VTK REQUIRED)
include(${VTK_USE_FILE})

add_executable(raycaster MACOSX_BUNDLE main.cxx Raycaster.cxx Vector.cxx)

if(VTK_LIBRARIES)
  target_link_libraries(raycaster ${VTK_LIBRARIES} ${CMAKE_THREAD_LIBS_INIT})
else()
  target_link_libraries(raycaster vtkHybrid ${CMAKE_THREAD_LIBS_INIT})
endif()


