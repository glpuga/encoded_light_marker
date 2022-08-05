cmake_minimum_required(VERSION 3.5)
project(elm_trackers)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)

add_library(
  ${PROJECT_NAME}
  src/datatypes/Buffer2D.cpp src/generators/TrackerCarrierGenerator.cpp
  src/overengineering/TrackerMarker.cpp
  src/overengineering/ToroidalTrackerDriver.cpp)

target_include_directories(${PROJECT_NAME} PUBLIC include)

ament_target_dependencies(${PROJECT_NAME})

install(TARGETS ${PROJECT_NAME} DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # set(ament_cmake_copyright_FOUND TRUE)
  # ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest)
  set(test_target_name ${PROJECT_NAME}-tests)

  file(GLOB slow_test_files_list tests/test_*.cpp)
  ament_add_gtest(${test_target_name} ${slow_test_files_list})
  target_link_libraries(${test_target_name} ${PROJECT_NAME})

endif()

ament_package()
