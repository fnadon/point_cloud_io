cmake_minimum_required(VERSION 3.5.1)
project(point_cloud_io)

set(CMAKE_CXX_STANDARD 17)
add_compile_options(-Wall -Wextra -Wpedantic -Werror=return-type)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Options.
option(BUILD_WITH_VTK_SUPPORT "Build package with support for VTK files." OFF)
if(BUILD_WITH_VTK_SUPPORT)
  message("point_cloud_io will be built with support for VTK files.")
  add_definitions(-DHAVE_VTK)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(sensor_msgs REQUIRED)

ament_export_dependencies(pcl_ros rclcpp sensor_msgs)
ament_export_include_directories(include)

include_directories(
  include
  SYSTEM
    ${catkin_INCLUDE_DIRS}
)

###########
## Build ##
###########
add_executable(read
  src/read_node.cpp
  src/Read.cpp
)
ament_target_dependencies(read rclcpp pcl_ros sensor_msgs)

add_executable(write
  src/write_node.cpp
  src/Write.cpp
)
ament_target_dependencies(write rclcpp pcl_ros sensor_msgs)


#############
## Install ##
#############
install(
  TARGETS
    read
    write
  DESTINATION lib/${PROJECT_NAME}
)
install(
  DIRECTORY include/
  DESTINATION include
)
ament_export_include_directories(include)
install(
  DIRECTORY
    examples
    launch
  DESTINATION share/${PROJECT_NAME}
)

find_package(cmake_clang_tools QUIET)
if(cmake_clang_tools_FOUND)
  add_default_clang_tooling(
    TARGETS
      read
      write
  )
endif(cmake_clang_tools_FOUND)

##############
##   Test   ##
##############
if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(test_${PROJECT_NAME}
    test/empty_test.cpp
  )
  add_dependencies(test_${PROJECT_NAME}
    read
    write
  )
  target_include_directories(test_${PROJECT_NAME} PRIVATE
    include
  )
  target_include_directories(test_${PROJECT_NAME} SYSTEM PUBLIC
    ${catkin_INCLUDE_DIRS}
  )
  target_link_libraries(test_${PROJECT_NAME}
    gtest_main
    ${catkin_LIBRARIES}
  )

  ###################
  ## Code_coverage ##
  ###################
  find_package(cmake_code_coverage QUIET)
  if(cmake_code_coverage_FOUND)
    add_gtest_coverage(
      TEST_BUILD_TARGETS
        test_${PROJECT_NAME}
    )
  endif()
endif()


ament_package()
