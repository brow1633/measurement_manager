cmake_minimum_required(VERSION 3.14)
project(measurement_manager VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

option(BUILD_TESTING "Build tests" OFF)

include(CTest)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include_directories(include)

add_executable(example example/main.cpp)

if(BUILD_TESTING)
    enable_testing()
    find_package(GTest REQUIRED)
    find_package(Threads REQUIRED)

    add_executable(test_measurement_manager
        test/test_measurement_manager.cpp)

    target_link_libraries(test_measurement_manager
    PRIVATE
        GTest::gtest
        GTest::gtest_main
    )
    include(GoogleTest)
    gtest_discover_tests(test_measurement_manager)

endif()
