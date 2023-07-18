cmake_minimum_required(VERSION 3.0.2)
project(uwb)
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  gazebo_msgs
  roscpp
  std_msgs
  uwb_msgs
)

find_package(Metis REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)
find_package(Boost REQUIRED COMPONENTS serialization)
find_package(Boost REQUIRED COMPONENTS filesystem)
find_package(Boost REQUIRED COMPONENTS thread)
find_package(Boost REQUIRED COMPONENTS date_time)
find_package(Boost REQUIRED COMPONENTS timer)
find_package(Boost REQUIRED COMPONENTS chrono)
find_package(fmt)
find_package(Ceres REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(GTSAM REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES uwb
#  CATKIN_DEPENDS gazebo_msgs roscpp std_msgs
#  DEPENDS system_lib
)

include_directories(
        ${catkin_INCLUDE_DIRS}
        ${CERES_INCLUDE_DIRS}
        ${Eigen_INCLUDE_DIRS}
        ${Boost_INCLUDE_DIRS}
        ${GTSAM_INCLUDE_DIR}
        ${METIS_INCLUDE_DIR}
)

add_executable(draft_node src/draft_node.cpp)
add_executable(uwb_sim src/uwb_sim.cpp)
add_executable(playground src/playground.cpp)

target_link_libraries(draft_node
        ${catkin_LIBRARIES}
        ${Boost_LIBRARIES}
        ${METIS_LIBRARIES}
        gtsam
)

target_link_libraries(uwb_sim
        ${catkin_LIBRARIES}
        ${Boost_LIBRARIES}
        ${METIS_LIBRARIES}
        gtsam
)

target_link_libraries(playground
        ${catkin_LIBRARIES}
        ${Boost_LIBRARIES}
        ${METIS_LIBRARIES}
        gtsam
)