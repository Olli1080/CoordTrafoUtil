find_package(GTest REQUIRED)

add_executable(transformation_tests 
    transformation_tests.cpp
    handedness_tests.cpp
)

target_link_libraries(transformation_tests 
    PRIVATE 
        base-transformation
        GTest::gtest_main
)

if (USE_EIGEN)
    target_compile_definitions(transformation_tests PRIVATE USE_EIGEN)
endif()

if (USE_PCL)
    target_compile_definitions(transformation_tests PRIVATE USE_PCL)
endif()

gtest_discover_tests(transformation_tests)
