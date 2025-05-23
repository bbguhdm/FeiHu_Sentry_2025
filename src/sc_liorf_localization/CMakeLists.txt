cmake_minimum_required(VERSION 3.5)
project(sc_liorf_localization)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g -pthread")

include(cmake/rosidl_get_typesupport_target.cmake)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(pcl_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)
find_package(GTSAM REQUIRED)
find_package(Eigen REQUIRED)
find_package(OpenMP REQUIRED)
find_package(Boost REQUIRED COMPONENTS timer)
find_package(livox_ros_driver2 REQUIRED)
find_package(rcpputils REQUIRED)
set(ros2_lib "rclcpp;rclpy;std_msgs;sensor_msgs;geometry_msgs;nav_msgs;cv_bridge;pcl_msgs;pcl_conversions;visualization_msgs;tf2;tf2_ros;PCL;tf2_eigen;tf2_sensor_msgs;tf2_geometry_msgs")

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "/usr/share/cmake/geographiclib/")
find_package(GeographicLib REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME} 
  "msg/CloudInfo.msg"
  "srv/SaveMap.srv"
  DEPENDENCIES std_msgs sensor_msgs
)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} rosidl_typesupport_cpp)

# include directories
include_directories(
  lib
	include
	${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
	${GTSAM_INCLUDE_DIR}
)

# link directories
link_directories(
  lib
	include
	${PCL_LIBRARY_DIRS}
  ${OpenCV_LIBRARY_DIRS}
  ${GTSAM_LIBRARY_DIRS}
)

file(GLOB common_lib "lib/*.cpp")

###########
## Build ##
###########
# <!-- sc_liorf_localization_yjz_lucky_boy -->

add_executable(${PROJECT_NAME}_featureExtraction src/featureExtraction.cpp)
ament_target_dependencies(${PROJECT_NAME}_featureExtraction ${ros2_lib}  livox_ros_driver2)
target_link_libraries(${PROJECT_NAME}_featureExtraction "${cpp_typesupport_target}") 

# Range Image Projection
add_executable(${PROJECT_NAME}_imageProjection src/imageProjection.cpp ${common_lib})
ament_target_dependencies(${PROJECT_NAME}_imageProjection ${ros2_lib} livox_ros_driver2)
target_link_libraries(${PROJECT_NAME}_imageProjection "${cpp_typesupport_target}" ${PCL_LIBRARIES} ${OpenCV_LIBRARIES})

# # Mapping Optimization
add_executable(${PROJECT_NAME}_mapOptmization src/mapOptmization.cpp ${common_lib} src/Scancontext.cpp)
target_compile_options(${PROJECT_NAME}_mapOptmization PRIVATE ${OpenMP_CXX_FLAGS})
ament_target_dependencies(${PROJECT_NAME}_mapOptmization ${ros2_lib} livox_ros_driver2)
target_link_libraries(${PROJECT_NAME}_mapOptmization Boost::timer "${cpp_typesupport_target}" ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} ${OpenMP_CXX_FLAGS} ${GeographicLib_LIBRARIES} gtsam)

# # IMU Preintegration
add_executable(${PROJECT_NAME}_imuPreintegration src/imuPreintegration.cpp ${common_lib})
ament_target_dependencies(${PROJECT_NAME}_imuPreintegration ${ros2_lib} livox_ros_driver2)
target_link_libraries(${PROJECT_NAME}_imuPreintegration Boost::timer "${cpp_typesupport_target}" ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} gtsam) 


#Global Localization
add_executable(${PROJECT_NAME}_localization src/localization.cpp ${common_lib}  src/Scancontext.cpp)
target_compile_options(${PROJECT_NAME}_localization PRIVATE ${OpenMP_CXX_FLAGS})
ament_target_dependencies(${PROJECT_NAME}_localization ${ros2_lib} livox_ros_driver2 )
target_link_libraries(${PROJECT_NAME}_localization Boost::timer "${cpp_typesupport_target}"  ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} ${OpenMP_CXX_FLAGS} ${GeographicLib_LIBRARIES} gtsam stdc++fs ndt_omp )

#Rm Localization
add_executable(${PROJECT_NAME}_rm_localization src/rm_localization.cpp ${common_lib}  src/Scancontext.cpp)
target_compile_options(${PROJECT_NAME}_rm_localization PRIVATE ${OpenMP_CXX_FLAGS})
ament_target_dependencies(${PROJECT_NAME}_rm_localization ${ros2_lib} livox_ros_driver2 )
target_link_libraries(${PROJECT_NAME}_rm_localization Boost::timer "${cpp_typesupport_target}"  ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} ${OpenMP_CXX_FLAGS} ${GeographicLib_LIBRARIES} gtsam stdc++fs ndt_omp )

# # Mapping Optimization
add_executable(${PROJECT_NAME}_sc_mapOptmization src/sc_mapOptmization.cpp ${common_lib} src/Scancontext.cpp)
target_compile_options(${PROJECT_NAME}_sc_mapOptmization PRIVATE ${OpenMP_CXX_FLAGS})
ament_target_dependencies(${PROJECT_NAME}_sc_mapOptmization ${ros2_lib} livox_ros_driver2)
target_link_libraries(${PROJECT_NAME}_sc_mapOptmization Boost::timer "${cpp_typesupport_target}" ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} ${OpenMP_CXX_FLAGS} ${GeographicLib_LIBRARIES} gtsam)

install(
  DIRECTORY launch config rviz 
  DESTINATION share/${PROJECT_NAME}/
)

install(
  TARGETS 
	${PROJECT_NAME}_featureExtraction
	${PROJECT_NAME}_imageProjection
	${PROJECT_NAME}_imuPreintegration
	${PROJECT_NAME}_mapOptmization
  ${PROJECT_NAME}_localization
  ${PROJECT_NAME}_sc_mapOptmization
  ${PROJECT_NAME}_rm_localization
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY "include/"
  DESTINATION include
)

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

ament_export_include_directories(include)

ament_package()
