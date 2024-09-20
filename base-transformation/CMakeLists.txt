add_library(base-transformation)

set(INC_DIR include/base-transformation)

target_include_directories(base-transformation PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/base-transformation>
    $<INSTALL_INTERFACE:include>
)

target_sources(base-transformation
    PRIVATE    
        src/TransformationHelper.cpp
    PUBLIC FILE_SET HEADERS
    BASE_DIRS ${INC_DIR}
    FILES
        ${INC_DIR}/concepts.h
        ${INC_DIR}/TransformationHelper.h
)

if (USE_EIGEN OR USE_PCL)
    add_subdirectory(Plugins)
endif()

target_compile_features(base-transformation PUBLIC cxx_std_20)

if (USE_EIGEN OR USE_PCL)
    target_link_libraries(base-transformation 
        PUBLIC
            plugins
    )
endif()

install(
    TARGETS 
        base-transformation
    EXPORT
        base-transformation-targets
    LIBRARY
        DESTINATION lib
    FILE_SET HEADERS
        DESTINATION include/base-transformation
)