cmake_minimum_required(VERSION 3.5)
project(libuvc_cam)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

find_package(libuvc REQUIRED)

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/libuvc_cam.cpp
  src/libuvc_cam_node.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC ${libuvc_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} ${libuvc_LIBRARIES})

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "libuvc_cam::UvcCameraNode"
  EXECUTABLE ${PROJECT_NAME}_node
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

install(FILES example.rules
  DESTINATION share/${PROJECT_NAME}
)

ament_auto_package(
  INSTALL_TO_SHARE
    config
    launch
)
