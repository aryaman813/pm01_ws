cmake_minimum_required(VERSION 3.8)
project(resource)

find_package(ament_cmake REQUIRED)

# Register the package name used in the robot's URDF. Its robot directory is
# linked to a read-only SFTP mount by scripts/use_orin_meshes.sh on the laptop.

ament_package()
