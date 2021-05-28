cmake_minimum_required(VERSION 2.8.3)
project(mod_path_planning)

set(CMAKE_BUILD_TYPE Debug)
set(MRPT_DONT_USE_DBG_LIBS 1)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "-m64")
set(CMAKE_C_FLAGS "-m64")
set(CMAKE_EXE_LINKER_FLAGS "-m64")
set(CMAKE_MODULE_LINKER_FLAGS "-m64")
set(CMAKE_SHARED_LINKER_FLAGS "-m64")

add_compile_options(-Wno-ignored-attributes -fext-numeric-literals)

# 1) OMPL
find_package(ompl 1.5.2 EXACT REQUIRED)
if(OMPL_FOUND)
  MESSAGE(STATUS "Found ompl ${OMPL_VERSION} at ${OMPL_INCLUDE_DIRS}")
endif(OMPL_FOUND)

# 2) MRPT
find_package(MRPT REQUIRED)

# 3) ROS
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  ompl_mod_objectives
  ompl_planners_ros
  std_msgs
  roscpp
  whytemap_ros
  cliffmap_ros
  stefmap_ros
  gmmtmap_ros
  nav_msgs
  costmap_2d
  visualization_msgs
  message_generation
)

# 4) Boost
find_package(Boost REQUIRED)

add_message_files(
  FILES
  MoDPlanningGoal.msg
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime std_msgs geometry_msgs ompl_mod_objectives roscpp whytemap_ros cliffmap_ros stefmap_ros gmmtmap_ros nav_msgs costmap_2d visualization_msgs ompl_planners_ros
  DEPENDS OMPL MRPT
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${OMPL_INCLUDE_DIRS}
  ${MRPT_INCLUDE_DIRS}
)


add_executable(mod_reeds_shepp_rrtstar_planner src/mod_reeds_shepp_rrtstar_planner.cpp)
add_dependencies(mod_reeds_shepp_rrtstar_planner ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(mod_reeds_shepp_rrtstar_planner ${catkin_LIBRARIES} ${OMPL_LIBRARIES} ${MRPT_LIBRARIES} ${Boost_LIBRARIES})