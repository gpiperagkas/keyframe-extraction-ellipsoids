/*
    Keyframe extraction with semantic graphs in assembly processes.
    Copyright (C) 2016-2025  Grigorios Piperagkas

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <https://www.gnu.org/licenses/>.
*/

project(semOAtrackxml_2hands_0.1)
cmake_minimum_required(VERSION 2.8)
set(CMAKE_BUILD_TYPE debug)
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR})
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED COMPONENTS thread)
find_package(PCL 1.7 REQUIRED)
include_directories(${CMAKE_SOURCE_DIR} 
		${CMAKE_BINARY_DIR} 
		${OpenCV_INCLUDE_DIRS}
		${EIGEN3_INCLUDE_DIR}
		${Boost_INCLUDE_DIR}
		${PCL_INCLUDE_DIRS}
)
link_directories(${PCL_LIBRARY_DIRS})
add_executable(main  pugixml-1.7/src/pugixml.cpp eventchain.cpp xmlparser.cpp xmlhandparser.cpp tracker.cpp imagergb.cpp ellipsoid.cpp main.cpp)
target_link_libraries(main m ${OpenCV_LIBS} ${Boost_LIBRARIES} ${PCL_LIBRARIES})
