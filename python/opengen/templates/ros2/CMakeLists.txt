cmake_minimum_required(VERSION 3.8)
project({{ros.package_name}})

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

# tells CMake's FindPython3 to prefer a venv if one is active
# (instead of the system-wide python)
set(Python3_FIND_VIRTUALENV FIRST)
if(NOT Python3_EXECUTABLE AND DEFINED ENV{VIRTUAL_ENV})
  set(_open_python3_executable "$ENV{VIRTUAL_ENV}/bin/python")
  if(EXISTS "${_open_python3_executable}")
    set(Python3_EXECUTABLE "${_open_python3_executable}")
  endif()
endif()
find_package(Python3 REQUIRED COMPONENTS Interpreter Development NumPy)
set(Python_EXECUTABLE ${Python3_EXECUTABLE})
set(Python_INCLUDE_DIRS ${Python3_INCLUDE_DIRS})
set(Python_LIBRARIES ${Python3_LIBRARIES})
set(Python_NumPy_INCLUDE_DIRS ${Python3_NumPy_INCLUDE_DIRS})
find_package(rosidl_default_generators REQUIRED)

set(msg_files
  "msg/OptimizationResult.msg"
  "msg/OptimizationParameters.msg"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
)

ament_export_dependencies(rosidl_default_runtime)

include_directories(
  ${PROJECT_SOURCE_DIR}/include
)

set(NODE_NAME {{ros.node_name}})
add_executable(${NODE_NAME} src/open_optimizer.cpp)
ament_target_dependencies(${NODE_NAME} rclcpp)
if(WIN32)
  set(OPEN_STATIC_LIB ${PROJECT_SOURCE_DIR}/extern_lib/{{meta.optimizer_name}}.lib)
else()
  set(OPEN_STATIC_LIB ${PROJECT_SOURCE_DIR}/extern_lib/lib{{meta.optimizer_name}}.a)
endif()
target_link_libraries(
  ${NODE_NAME}
  ${OPEN_STATIC_LIB}
)
if(NOT WIN32)
  target_link_libraries(
    ${NODE_NAME}
    m
    dl
  )
endif()
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(${NODE_NAME} "${cpp_typesupport_target}")

install(TARGETS
  ${NODE_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  config
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
