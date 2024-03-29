cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_STANDARD 17)

# Find ROS
find_package(roscpp REQUIRED)
find_package(std_msgs REQUIRED)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        rospy
        std_msgs
        )

include_directories(${roscpp_INCLUDE_DIRS} ${CATKIN_INCLUDE_DIRS})
include_directories(${std_msgs_INCLUDE_DIRS})

# Find Gazebo
find_package(gazebo REQUIRED)
include_directories(${GAZEBO_INCLUDE_DIRS})
include_directories(${EIGEN3_INCLUDE_DIR})

link_directories(${GAZEBO_LIBRARY_DIRS})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

# Build our plugin
add_library(suv_plugin SHARED src/suv_plugin.cc)
target_link_libraries(suv_plugin ${GAZEBO_libraries} ${roscpp_LIBRARIES} ${catkin_LIBRARIES})

# Eigen
set(EIGEN3_INCLUDE_DIR "$ENV{EIGEN3_INCLUDE_DIR}")
if (NOT EIGEN3_INCLUDE_DIR)
    message(FATAL_ERROR "Please point the environment variable EIGEN3_INCLUDE_DIR to the include directory of your Eigen3 installation.")
endif ()
include_directories(${EIGEN3_INCLUDE_DIR})
target_include_directories(suv_plugin PUBLIC ${EIGEN3_INCLUDE_DIR})
#message("EIGEN=${EIGEN3_INCLUDE_DIR}")
#message("cmake_module_path: " ${CMAKE_MODULE_PATH})

# Build the stand-alone test program
add_executable(vel src/vel_controller.cc)
target_link_libraries(vel ${catkin_LIBRARIES})

if (${gazebo_VERSION_MAJOR} LESS 6)
    # These two
    include(FindBoost)
    find_package(Boost ${MIN_BOOST_VERSION} REQUIRED system filesystem regex)
    target_link_libraries(vel ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES})
else ()
    target_link_libraries(vel ${GAZEBO_LIBRARIES})
endif ()


