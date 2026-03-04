# Example: Unity to Unreal conversion using Eigen
if (USE_EIGEN)
    add_executable(example_unity_to_unreal unity_to_unreal.cpp)
    target_link_libraries(example_unity_to_unreal PRIVATE Transformation::Plugins)
endif()

# Example: Custom Type Support
add_executable(example_custom_type custom_type_support.cpp)
target_link_libraries(example_custom_type PRIVATE Transformation::Core)
