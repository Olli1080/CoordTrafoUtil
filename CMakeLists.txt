cmake_minimum_required(VERSION 3.23)

project(TransformationBase 
    LANGUAGES CXX)

include(GNUInstallDirs)

option(USE_EIGEN "Build with eigen plugin" ON)
option(USE_PCL "Build with pcl plugin" ON)

add_subdirectory(transformationBase)

include(CMakePackageConfigHelpers)
configure_package_config_file(
  "${CMAKE_CURRENT_SOURCE_DIR}/base-transformation-config.cmake.in"
  "${CMAKE_CURRENT_BINARY_DIR}/base-transformation-config.cmake"
  INSTALL_DESTINATION share/base-transformation
)

install(FILES
          "${CMAKE_CURRENT_BINARY_DIR}/base-transformation-config.cmake"
        DESTINATION share/base-transformation)

install(
  EXPORT base-transformation-targets
  FILE base-transformation-targets.cmake
  DESTINATION share/base-transformation
  NAMESPACE base-transformation::
)