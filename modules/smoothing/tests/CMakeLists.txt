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

set(SMOOTHING_DRIVER_SRC "${CMAKE_SOURCE_DIR}/modules/smoothing/src/driver")

add_executable(smoothing_tests
    ${CMAKE_CURRENT_SOURCE_DIR}/test_skeletal_smoothing.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/test_smoothing_recording.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/test_prediction_ui_logic.cpp
    ${SMOOTHING_DRIVER_SRC}/SkeletalSmoothingMath.cpp
    ${CMAKE_SOURCE_DIR}/core/src/driver/ServerTrackedDeviceProviderConfigPacking.cpp
)

target_include_directories(smoothing_tests
    PRIVATE ${CMAKE_SOURCE_DIR}
    PRIVATE ${SMOOTHING_DRIVER_SRC}
    PRIVATE ${CMAKE_SOURCE_DIR}/modules/smoothing/src/overlay
    PRIVATE ${CMAKE_SOURCE_DIR}/core/src/common
    PRIVATE ${CMAKE_SOURCE_DIR}/core/src/driver
    PRIVATE ${CMAKE_SOURCE_DIR}/lib
    PRIVATE ${CMAKE_SOURCE_DIR}/lib/openvr/headers
)

target_link_libraries(smoothing_tests
    PRIVATE GTest::gtest_main
)

target_compile_definitions(smoothing_tests
    PRIVATE NOMINMAX
    PRIVATE UNICODE
    PRIVATE _UNICODE
    PRIVATE SMOOTHING_TESTS=1
)

if(MSVC)
    target_compile_options(smoothing_tests PRIVATE /W4 /wd4100)
endif()

adjust_bin_paths(smoothing_tests)

include(GoogleTest)
gtest_discover_tests(smoothing_tests DISCOVERY_TIMEOUT 30)

set_property(TARGET smoothing_tests PROPERTY FOLDER "tests")
