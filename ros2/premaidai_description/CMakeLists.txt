cmake_minimum_required(VERSION 3.8)
project(premaidai_description)

find_package(ament_cmake REQUIRED)

# meshes and the plain URDF are shared with the rest of the repository (built in-tree: colcon build from the repository root)
set(REPO ${CMAKE_CURRENT_SOURCE_DIR}/../..)
install(DIRECTORY ${REPO}/meshes DESTINATION share/${PROJECT_NAME})
install(FILES ${REPO}/urdf/premaidai.urdf DESTINATION share/${PROJECT_NAME}/urdf)
install(DIRECTORY urdf config launch rviz DESTINATION share/${PROJECT_NAME})

ament_package()
