cmake_minimum_required(VERSION 3.22)
project(aerial_demo_mas VERSION 0.0.1 LANGUAGES CXX)

#============================================================================
# Configure the project
#============================================================================
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Werror=format-security -Wpedantic)
endif()

if(LOTUS_TIDY)
  set(CMAKE_CXX_CLANG_TIDY
    "clang-tidy;-checks=-*,clang-analyzer-*,modernize-*,performance-*,hicpp-*,concurrency-*,bugprone-*,portability-*,cppcoreguidelines-*;-header-filter=.*")
endif()

#============================================================================
# Set project-specific options
#============================================================================
if(DEBUG)
  message(STATUS "Building ${PROJECT_NAME} with debug options ${DEBUG}")
  add_compile_options(-g -Og)
  find_package(backward_ros)
  add_definitions(-DDEBUG=${DEBUG})
endif()

if(BUILD_PROFILING)
  add_compile_options(-pg -g -Og)
endif()

# AddressSanitizer
if(LOTUS_ASAN)
  message(STATUS "Building ${PROJECT_NAME} with AddressSanitizer enabled")
  add_compile_options(-fsanitize=address -fno-omit-frame-pointer -g)
  add_link_options(-fsanitize=address)
endif()

#============================================================================
# Search for dependencies
#============================================================================
find_package(ament_cmake REQUIRED)
find_package(lotusim_mas REQUIRED)

#============================================================================
# Create library
#============================================================================
add_library(aerial_demo_mas SHARED 
  src/aerial_mas.cpp
)

target_include_directories(aerial_demo_mas PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(aerial_demo_mas
  lotusim_mas
)


#============================================================================
# Installation
#============================================================================
install(
  DIRECTORY include/
  DESTINATION include
  FILES_MATCHING
  PATTERN "*.hpp"
  PATTERN "*.h"
)

install(
  TARGETS aerial_demo_mas
  EXPORT export_aerial_demo_mas
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

#============================================================================
# Export
#============================================================================
ament_export_targets(export_aerial_demo_mas HAS_LIBRARY_TARGET)
ament_export_dependencies(lotusim_mas)

ament_package()