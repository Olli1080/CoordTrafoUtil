cmake_minimum_required(VERSION 3.23)

project(TransformationBase
    VERSION 0.1.0
    LANGUAGES CXX)

include(GNUInstallDirs)

option(USE_EIGEN "Build with eigen plugin" ON)
option(USE_PCL "Build with pcl plugin" ON)

option(BUILD_TESTS "Build tests" OFF)
option(BUILD_EXAMPLES "Build examples" OFF)

add_subdirectory(base-transformation)

if (BUILD_TESTS)
  enable_testing()
  add_subdirectory(tests)
endif()

if (BUILD_EXAMPLES)
  add_subdirectory(examples)
endif()

include(CMakePackageConfigHelpers)
configure_package_config_file(
  "${CMAKE_CURRENT_SOURCE_DIR}/base-transformation-config.cmake.in"
  "${CMAKE_CURRENT_BINARY_DIR}/base-transformation-config.cmake"
  INSTALL_DESTINATION share/base-transformation
)

write_basic_package_version_file(
  "${CMAKE_CURRENT_BINARY_DIR}/base-transformation-config-version.cmake"
  VERSION "${PROJECT_VERSION}"
  COMPATIBILITY SameMajorVersion
  ARCH_INDEPENDENT
)

install(FILES
          "${CMAKE_CURRENT_BINARY_DIR}/base-transformation-config.cmake"
          "${CMAKE_CURRENT_BINARY_DIR}/base-transformation-config-version.cmake"
        DESTINATION share/base-transformation)

install(
  EXPORT base-transformation-targets
  FILE base-transformation-targets.cmake
  DESTINATION share/base-transformation
  NAMESPACE Transformation::
)