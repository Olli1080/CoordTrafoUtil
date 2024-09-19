add_library(plugins)

set(INC_DIR include/base-transformation/Plugin)

target_include_directories(plugins PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/base-transformation/include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/base-transformation/Plugin>
    $<INSTALL_INTERFACE:include>
)

target_sources(plugins
    PUBLIC FILE_SET HEADERS 
    BASE_DIRS ${INC_DIR}
    FILES
        ${INC_DIR}/plugins.h
)

if (USE_EIGEN)
    find_package(Eigen3 REQUIRED NO_MODULE)
    target_compile_definitions(plugins PUBLIC USE_EIGEN)
    target_link_libraries(plugins PUBLIC Eigen3::Eigen)

    target_sources(plugins
    PRIVATE
        src/eigen.cpp
    PUBLIC FILE_SET HEADERS 
    BASE_DIRS ${INC_DIR}
    FILES
        ${INC_DIR}/eigen.h
    )
endif()
if (USE_PCL)
    find_package(PCL COMPONENTS common)
    target_compile_definitions(plugins PUBLIC USE_PCL)
    target_link_libraries(plugins PUBLIC ${PCL_LIBRARIES})

    target_sources(plugins
    PRIVATE
        src/pcl.cpp
    PUBLIC FILE_SET HEADERS 
    BASE_DIRS ${INC_DIR}
    FILES
        ${INC_DIR}/pcl.h
    )
endif()

target_compile_features(plugins PUBLIC cxx_std_20)

install(
    TARGETS 
        plugins
    EXPORT
        base-transformation-targets
    LIBRARY
        DESTINATION lib
    FILE_SET HEADERS
        DESTINATION include/base-transformation
)