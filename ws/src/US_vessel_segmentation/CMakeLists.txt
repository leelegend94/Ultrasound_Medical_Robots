cmake_minimum_required(VERSION 3.2.0)
#set (CMAKE_CXX_STANDARD 14)

project(vessel_segmentation)

find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  roscpp
  geometry_msgs
  std_msgs
  sensor_msgs
  std_srvs
  message_generation
)

find_package(OpenCV REQUIRED)

find_package (Eigen3 REQUIRED)

#find_package(Threads REQUIRED)

## Messages.
add_service_files(FILES
                  ultrasoundFrame.srv
                  )

generate_messages(DEPENDENCIES std_msgs sensor_msgs geometry_msgs)


catkin_package(INCLUDE_DIRS include
			CATKIN_DEPENDS roscpp std_srvs message_runtime
)

# Define and configure the targets
set(Sources
	src/frameServer.cpp
	src/segmenter.cpp
)
set(HEADERS
	include/segmenter.h
)

include_directories(include
  ${catkin_INCLUDE_DIRS}
	${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
	)



# Define target library
add_library(vessel_segmentation_lib SHARED ${Sources} ${HEADERS})
target_include_directories(vessel_segmentation_lib PRIVATE
	${CMAKE_CURRENT_SOURCE_DIR}
	${CMAKE_CURRENT_BINARY_DIR}			# needed for the UI files
)
# Link agains the ImFusionLib and selected modules/plugins
#set(vessel_segmentation_TARGET_LIBS
#	${OpenCV_LIBS}
#	)
#target_link_libraries(VesselSegmentation ${vessel_segmentation_TARGET_LIBS})
