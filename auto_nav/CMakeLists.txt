cmake_minimum_required(VERSION 3.5)
project(auto_nav)

#set vcpkg toolchain file in order to import xtensor package via vcpkg package manager.
if(DEFINED ENV{VCPKG_ROOT})
    set(CMAKE_TOOLCHAIN_FILE "$ENV{VCPKG_ROOT}/scripts/buildsystems/vcpkg.cmake"
        CACHE STRING "Vcpkg toolchain file")
endif()

set(xtensor_DIR "$ENV{VCPKG_ROOT}/installed/arm64-linux/share/xtensor")
set(xtl_DIR "$ENV{VCPKG_ROOT}/installed/arm64-linux/share/xtl")
set(nlohmann_json_DIR "$ENV{VCPKG_ROOT}/installed/arm64-linux/share/nlohmann_json")
# set(OPENCV_DIR "/opt/ros/foxy/share/opencv4")



# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O3)
endif()


# find dependencies
find_package(ament_cmake REQUIRED)
find_package(OpenCV 4.2.0 REQUIRED)
find_package(realsense2 REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(xtensor CONFIG REQUIRED)




#uncomment the following section in order to fill in
# further dependencies manually.
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(ackermann_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(vesc_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(image_transport REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)


include_directories( ${OpenCV_INCLUDE_DIRS}
  /usr/local/include/QuadProg++
  include
  ${xtensor_INCLUDE_DIRS}
)


add_executable(navigation src/navigation.cpp)
add_executable(experiment src/experiment.cpp)

ament_target_dependencies(experiment
      rclcpp
      std_msgs
      ackermann_msgs
      sensor_msgs
      vesc_msgs   
      nav_msgs   
      cv_bridge
      image_transport

      visualization_msgs
      geometry_msgs
)




ament_target_dependencies(navigation
      rclcpp
      std_msgs
      ackermann_msgs
      sensor_msgs   
      nav_msgs   
      OpenCV 
      rmw
      rcutils

      cv_bridge
      image_transport
      visualization_msgs
      geometry_msgs
)

file(GLOB NODE_SRC_FILES node/*.cpp)

foreach(_node_file ${NODE_SRC_FILES})
  get_filename_component(_node_name ${_node_file} NAME_WE)
  add_executable(${_node_name} ${_node_file})
  target_link_libraries(${_node_name} ${LIBS})
endforeach()

find_library(QUADPROGPP_LIBRARIES NAMES quadprog)
find_path(QUADPROGPP_INCLUDE_DIR NAMES QuadProg++.hh)
target_link_libraries(navigation ${QUADPROGPP_LIBRARIES} -lquadprog)
target_link_libraries(navigation ${OPENCV_LIBS})
target_link_libraries(navigation xtensor realsense2)


install(TARGETS navigation DESTINATION lib/${PROJECT_NAME})
install(TARGETS experiment DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)

install(DIRECTORY launch/
        DESTINATION share/${PROJECT_NAME}/launch
        
)

install(DIRECTORY config/
        DESTINATION share/${PROJECT_NAME}/config

)

ament_export_include_directories(include)
ament_export_include_directories(std_msgs)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
