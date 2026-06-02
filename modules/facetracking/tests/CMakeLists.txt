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

set(FT_DRIVER_SRC "${CMAKE_SOURCE_DIR}/modules/facetracking/src/driver")

add_executable(facetracking_tests
    ${CMAKE_CURRENT_SOURCE_DIR}/calibration_tests.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/vergence_tests.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/eyelid_sync_tests.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/feature_flags_tests.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/osc_publish_tests.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/upstream_shape_map_tests.cpp
    ${FT_DRIVER_SRC}/CalibrationEngine.cpp
    ${FT_DRIVER_SRC}/VergenceLock.cpp
    ${FT_DRIVER_SRC}/EyelidSync.cpp
    ${FT_DRIVER_SRC}/FaceOscAddressFilter.cpp
)

target_include_directories(facetracking_tests
    PRIVATE ${CMAKE_SOURCE_DIR}
    PRIVATE ${FT_DRIVER_SRC}
    PRIVATE ${CMAKE_SOURCE_DIR}/core/src/common
    PRIVATE ${CMAKE_SOURCE_DIR}/core/src/driver
    PRIVATE ${CMAKE_SOURCE_DIR}/lib
    PRIVATE ${CMAKE_SOURCE_DIR}/lib/openvr/headers
)

target_link_libraries(facetracking_tests PRIVATE GTest::gtest_main)

target_compile_definitions(facetracking_tests PRIVATE
    NOMINMAX
    UNICODE
    FACETRACKING_TESTS=1)

if(MSVC)
    target_compile_options(facetracking_tests PRIVATE /bigobj)
endif()

adjust_bin_paths(facetracking_tests)

include(GoogleTest)
gtest_discover_tests(facetracking_tests)

set_property(TARGET facetracking_tests PROPERTY FOLDER "tests")
