cmake_minimum_required(VERSION 3.8)
project(resource)

find_package(ament_cmake REQUIRED)

# Register the package name used in the robot's URDF, and install the meshes
# it references (package://resource/robot/pm_v2/meshes/*.STL) from the local
# MuJoCo assets, so RViz can draw the robot's /robot_description.
install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/../simulation/mujoco/assets/resource/robot/pm_v2/meshes
  DESTINATION share/${PROJECT_NAME}/robot/pm_v2
)

ament_package()
