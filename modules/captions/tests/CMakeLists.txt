cmake_minimum_required(VERSION 3.14)

include(FetchContent)
FetchContent_Declare(
    googletest
    URL https://github.com/google/googletest/archive/refs/tags/v1.14.0.zip
)
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(TR_HOST_SRC "${CMAKE_SOURCE_DIR}/modules/captions/src/host")

add_executable(captions_tests
    ${CMAKE_CURRENT_SOURCE_DIR}/captions_tests.cpp
    # Pull in the ChatboxPacer implementation for the pacer tests.
    ${TR_HOST_SRC}/ChatboxPacer.cpp)

target_include_directories(captions_tests PRIVATE
    ${TR_HOST_SRC}
    ${CMAKE_SOURCE_DIR}/modules/captions/src/overlay
    ${CMAKE_SOURCE_DIR}/core/src/common
    ${CMAKE_SOURCE_DIR}/lib
    ${CMAKE_SOURCE_DIR}/lib/openvr/headers)

target_link_libraries(captions_tests PRIVATE GTest::gtest_main)

target_compile_definitions(captions_tests PRIVATE
    NOMINMAX
    UNICODE
    CAPTIONS_TESTS=1)

if(MSVC)
    target_compile_options(captions_tests PRIVATE /bigobj)
endif()

adjust_bin_paths(captions_tests)

include(GoogleTest)
gtest_discover_tests(captions_tests)

set_property(TARGET captions_tests PROPERTY FOLDER "tests")
