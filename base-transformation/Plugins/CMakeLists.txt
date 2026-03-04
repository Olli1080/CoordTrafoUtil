add_library(base-transformation-plugins INTERFACE)
add_library(Transformation::Plugins ALIAS base-transformation-plugins)

set(INC_DIR include/base-transformation/Plugins)

target_include_directories(base-transformation-plugins INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/base-transformation/include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/base-transformation/Plugins>
    $<INSTALL_INTERFACE:include>
)

target_sources(base-transformation-plugins
    INTERFACE FILE_SET HEADERS 
    BASE_DIRS include/base-transformation
    FILES
        ${INC_DIR}/plugins.h
)

target_link_libraries(base-transformation-plugins INTERFACE Transformation::Core)

if (USE_EIGEN)
    find_package(Eigen3 REQUIRED NO_MODULE)
    target_compile_definitions(base-transformation-plugins INTERFACE USE_EIGEN)
    target_link_libraries(base-transformation-plugins INTERFACE Eigen3::Eigen)

    target_sources(base-transformation-plugins
    INTERFACE FILE_SET HEADERS 
    BASE_DIRS include/base-transformation
    FILES
        ${INC_DIR}/eigen.h
    )
endif()
if (USE_PCL)
    find_package(PCL COMPONENTS common)
    target_compile_definitions(base-transformation-plugins INTERFACE USE_PCL)
    target_link_libraries(base-transformation-plugins INTERFACE ${PCL_LIBRARIES})

    target_sources(base-transformation-plugins
    INTERFACE FILE_SET HEADERS 
    BASE_DIRS include/base-transformation
    FILES
        ${INC_DIR}/pcl.h
    )
endif()

install(
    TARGETS 
        base-transformation-plugins
    EXPORT
        base-transformation-targets
    FILE_SET HEADERS
        DESTINATION include/base-transformation
)
