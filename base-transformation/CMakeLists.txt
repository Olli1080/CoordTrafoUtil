add_library(base-transformation-core INTERFACE)
add_library(Transformation::Core ALIAS base-transformation-core)

set(INC_DIR include/base-transformation)

target_include_directories(base-transformation-core INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_sources(base-transformation-core
    INTERFACE FILE_SET HEADERS
    BASE_DIRS include
    FILES
        ${INC_DIR}/concepts.h
        ${INC_DIR}/traits.h
        ${INC_DIR}/TransformationHelper.h
)

if (USE_EIGEN OR USE_PCL)
    add_subdirectory(Plugins)
endif()

target_compile_features(base-transformation-core INTERFACE cxx_std_20)

install(
    TARGETS 
        base-transformation-core
    EXPORT
        base-transformation-targets
    FILE_SET HEADERS
        DESTINATION include/base-transformation
)
