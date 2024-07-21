find_package(PCL REQUIRED COMPONENTS common)
find_package (Eigen3 REQUIRED NO_MODULE)

add_library(base_transformation)

target_sources(base_transformation
    PUBLIC    
        TransformationHelper.cpp
        Plugins/eigen.cpp
        Plugins/pcl.cpp
    PUBLIC FILE_SET HEADERS FILES
        concepts.h
        TransformationHelper.h
        Plugins/eigen.h
        Plugins/pcl.h
        Plugins/plugins.h
)

message(PCL_COMMON_INCLUDE_DIR ${PCL_COMMON_INCLUDE_DIR})

target_compile_features(base_transformation PUBLIC cxx_std_20)
target_link_libraries(base_transformation INTERFACE ${PCL_COMMON_LIBRARIES} Eigen3::Eigen)
target_compile_definitions(base_transformation INTERFACE ${PCL_DEFINITIONS})
target_include_directories(base_transformation PUBLIC ${PCL_COMMON_INCLUDE_DIR} ${EIGEN3_INCLUDE_DIR})

install(TARGETS 
        base_transformation
    LIBRARY
        DESTINATION lib
    FILE_SET HEADERS
        DESTINATION base-transformation/include
    )