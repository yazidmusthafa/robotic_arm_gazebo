cmake_minimum_required(VERSION 3.22)
project(robotic_arm_v3)

# find dependencies
set(HW_IF_INCLUDE_DEPENDS
  pluginlib
  rcpputils
  hardware_interface
)

foreach(Dependency IN ITEMS ${HW_IF_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_visual_tools REQUIRED)                        #Added##
find_package(moveit_ros_planning_interface REQUIRED)   #Added##
find_package(moveit_task_constructor_core REQUIRED)    #Added##

#Added##
add_executable(robotic_arm_moveit src/robotic_arm_moveit.cpp)
ament_target_dependencies(robotic_arm_moveit moveit_visual_tools moveit_ros_planning_interface rclcpp)

add_executable(linear_movement src/linear_movement.cpp)
ament_target_dependencies(linear_movement moveit_task_constructor_core moveit_ros_planning_interface rclcpp)

add_executable(cartesian_planning src/cartesian_planning.cpp)
ament_target_dependencies(cartesian_planning moveit_visual_tools moveit_ros_planning_interface rclcpp)
#####

#Added##


add_library(
  robotic_arm_v3
  SHARED
  hardware/robotic_arm_hardware.cpp
)

target_compile_features(robotic_arm_v3 PUBLIC cxx_std_17)
target_include_directories(robotic_arm_v3 PUBLIC
$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/hardware/include>
$<INSTALL_INTERFACE:include/robotic_arm_v3>
)

ament_target_dependencies(
  robotic_arm_v3 PUBLIC
  ${HW_IF_INCLUDE_DEPENDS}
)

pluginlib_export_plugin_description_file(hardware_interface robotic_arm_v3.xml)     ## Needed for the Hardware interface to be seen as a plugin

install(
  DIRECTORY hardware/include/
  DESTINATION include/robotic_arm_v3
)

install(TARGETS robotic_arm_v3
  EXPORT export_robotic_arm_v3
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS
  cartesian_planning
  linear_movement
  robotic_arm_moveit
  DESTINATION lib/${PROJECT_NAME}) 
#####   

install(
  DIRECTORY launch src models meshes
  DESTINATION share/${PROJECT_NAME}
  PATTERN "setup_assistant.launch" EXCLUDE)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(FILES .setup_assistant DESTINATION share/${PROJECT_NAME})

ament_export_targets(export_robotic_arm_v3 HAS_LIBRARY_TARGET)
ament_export_dependencies(${HW_IF_INCLUDE_DEPENDS})
ament_package()
