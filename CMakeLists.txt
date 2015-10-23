cmake_minimum_required(VERSION 2.8.3)
project(infotaxis)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp tf tf2 tf2_ros move_base_msgs actionlib
)
find_package(PNG)


if (NOT INFOTAXIS_NOPNG)
	if (PNG_FOUND)
		include_directories(${PNG_INCLUDE_DIRS})
	else (PNG_FOUND)
		message("Can't find libpng")
		SET(INFOTAXIS_NOPNG 'true')
	endif(PNG_FOUND)
endif (NOT INFOTAXIS_NOPNG)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES infotaxis
  CATKIN_DEPENDS roscpp actionlib move_base_msgs tf2
#  DEPENDS system_lib
)


add_library(libinfotaxis STATIC IMPORTED)
set_target_properties(libinfotaxis PROPERTIES
    IMPORTED_LOCATION ${CMAKE_CURRENT_SOURCE_DIR}/lib/libinfotaxis.a)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
  "./include/"
)

## Declare a cpp library
# add_library(infotaxis
#   src/${PROJECT_NAME}/infotaxis.cpp
# )

## Declare a cpp executable
add_executable(infotaxis_node src/infotaxis_node.cpp)

set_property(TARGET infotaxis_node PROPERTY CXX_STANDARD 11)
set_property(TARGET infotaxis_node PROPERTY CXX_STANDARD_REQUIRED true)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(infotaxis_node infotaxis_generate_messages_cpp)

message(${catkin_LIBRARIES})

## Specify libraries to link a library or executable target against
target_link_libraries(infotaxis_node
	${catkin_LIBRARIES}
	libinfotaxis
)

if (NOT INFOTAXIS_NOPNG)
	target_link_libraries(infotaxis_node ${PNG_LIBRARIES})
endif (NOT INFOTAXIS_NOPNG)
