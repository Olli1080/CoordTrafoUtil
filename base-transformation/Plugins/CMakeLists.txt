# Configure the plugins.h header to bake in the build-time selection
configure_file(
    include/base-transformation/Plugins/plugins.h.in
    ${CMAKE_CURRENT_BINARY_DIR}/include/base-transformation/Plugins/plugins.h
)

add_library(base-transformation-plugins INTERFACE)
add_library(Transformation::Plugins ALIAS base-transformation-plugins)

target_include_directories(base-transformation-plugins INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/base-transformation/include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/include> # For the generated plugins.h
    $<INSTALL_INTERFACE:include>
)

# Export the generated plugins.h
target_sources(base-transformation-plugins
    INTERFACE FILE_SET HEADERS 
    BASE_DIRS ${CMAKE_CURRENT_BINARY_DIR}/include
    FILES
        ${CMAKE_CURRENT_BINARY_DIR}/include/base-transformation/Plugins/plugins.h
)

target_link_libraries(base-transformation-plugins INTERFACE Transformation::Core)

if (USE_EIGEN)
    find_package(Eigen3 REQUIRED NO_MODULE)
    target_link_libraries(base-transformation-plugins INTERFACE Eigen3::Eigen)

    target_sources(base-transformation-plugins
    INTERFACE FILE_SET HEADERS 
    BASE_DIRS include
    FILES
        include/base-transformation/Plugins/eigen.h
    )
endif()

if (USE_PCL)
    find_package(PCL COMPONENTS common)
    target_link_libraries(base-transformation-plugins INTERFACE ${PCL_LIBRARIES})

    target_sources(base-transformation-plugins
    INTERFACE FILE_SET HEADERS 
    BASE_DIRS include
    FILES
        include/base-transformation/Plugins/pcl.h
    )
endif()

set_target_properties(base-transformation-plugins PROPERTIES EXPORT_NAME Plugins)

install(
    TARGETS 
        base-transformation-plugins
    EXPORT
        base-transformation-targets
    FILE_SET HEADERS
        DESTINATION include
)
