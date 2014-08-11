cmake_minimum_required(VERSION 2.8)

PROJECT(SIFTICGM)

INCLUDE(FindPkgConfig)

pkg_check_modules(OPENCV REQUIRED opencv)
pkg_check_modules(GTK REQUIRED gtk+-2.0)

INCLUDE_DIRECTORIES(${CMAKE_CURRENT_BINARY_DIR}
                    ${CMAKE_CURRENT_SOURCE_DIR}
                    ${GTK_INCLUDE_DIRS} 
                    ${OPENCV_INCLUDE_DIRS}
                    )

link_directories(${GTK_LIBRARY_DIRS} 
                 ${OPENCV_LIBRARY_DIRS})

LINK_LIBRARIES(${GTK_LIBRARIES} 
               ${OPENCV_LIBRARIES})

ADD_EXECUTABLE(SIFTICGM 
               utils.c utils.h
               sift.c sift.h
               imgfeatures.c imgfeatures.h
               kdtree.c kdtree.h
               minpq.c minpq.h
               xform.c xform.h
               main.cpp
              )

SET(CMAKE_CXX_FLAGS " -march=core2 -pipe -fopenmp")
