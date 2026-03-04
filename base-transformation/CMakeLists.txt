add_library(base-transformation-core)
add_library(Transformation::Core ALIAS base-transformation-core)

set(INC_DIR include/base-transformation)

target_include_directories(base-transformation-core PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_sources(base-transformation-core
    PRIVATE    
        src/TransformationHelper.cpp
    PUBLIC FILE_SET HEADERS
    BASE_DIRS include
    FILES
        ${INC_DIR}/concepts.h
        ${INC_DIR}/traits.h
        ${INC_DIR}/TransformationHelper.h
)

if (USE_EIGEN OR USE_PCL)
    add_subdirectory(Plugins)
endif()

target_compile_features(base-transformation-core PUBLIC cxx_std_20)

install(
    TARGETS 
        base-transformation-core
    EXPORT
        base-transformation-targets
    LIBRARY
        DESTINATION lib
    FILE_SET HEADERS
        DESTINATION include/base-transformation
)
