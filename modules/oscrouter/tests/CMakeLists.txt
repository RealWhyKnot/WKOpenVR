cmake_minimum_required(VERSION 3.14)

include(FetchContent)
FetchContent_Declare(
    googletest
    URL https://github.com/google/googletest/archive/refs/tags/v1.14.0.zip
)
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(OR_DRIVER_SRC "${CMAKE_SOURCE_DIR}/modules/oscrouter/src/driver")
set(OR_OVERLAY_SRC "${CMAKE_SOURCE_DIR}/modules/oscrouter/src/overlay")

add_executable(oscrouter_tests
    ${CMAKE_CURRENT_SOURCE_DIR}/router_tests.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/test_oscrouter_ui_logic.cpp)

target_include_directories(oscrouter_tests
    PRIVATE ${OR_DRIVER_SRC}
    PRIVATE ${OR_OVERLAY_SRC}
    PRIVATE ${CMAKE_SOURCE_DIR}/core/src/common
    PRIVATE ${CMAKE_SOURCE_DIR}/lib
    PRIVATE ${CMAKE_SOURCE_DIR}/lib/openvr/headers)

target_link_libraries(oscrouter_tests PRIVATE GTest::gtest_main)

target_compile_definitions(oscrouter_tests PRIVATE
    NOMINMAX
    UNICODE
    OSCROUTER_TESTS=1)

if(MSVC)
    target_compile_options(oscrouter_tests PRIVATE /bigobj)
endif()

adjust_bin_paths(oscrouter_tests)

include(GoogleTest)
gtest_discover_tests(oscrouter_tests)

set_property(TARGET oscrouter_tests PROPERTY FOLDER "tests")
