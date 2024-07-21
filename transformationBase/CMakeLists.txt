find_package(PCL REQUIRED COMPONENTS common)
find_package (Eigen3 REQUIRED NO_MODULE)

add_library(transformation_base)

target_sources(transformation_base
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

target_compile_features(transformation_base PUBLIC cxx_std_20)
target_link_libraries(transformation_base INTERFACE ${PCL_COMMON_LIBRARIES} Eigen3::Eigen)
target_compile_definitions(transformation_base INTERFACE ${PCL_DEFINITIONS})
target_include_directories(transformation_base PUBLIC ${PCL_COMMON_INCLUDE_DIR} ${EIGEN3_INCLUDE_DIR})

install(TARGETS 
        transformation_base
    LIBRARY
        DESTINATION lib
    FILE_SET HEADERS
    )