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

set(IH_OVERLAY_SRC "${CMAKE_SOURCE_DIR}/modules/inputhealth/src/overlay")
set(IH_DRIVER_SRC  "${CMAKE_SOURCE_DIR}/modules/inputhealth/src/driver")

add_executable(inputhealth_tests
    ${CMAKE_CURRENT_SOURCE_DIR}/test_path_classifier.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/test_trigger_remap.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/test_stick_deadzone.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/test_inputhealth_config_roundtrip.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/test_profile_prune.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/test_presence_counting.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/test_ui_logic.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/test_learning_rules.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/test_button_bounce.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/stubs.cpp
    # Config round-trip tests compile Config.cpp directly to avoid pulling in the
    # full overlay (ImGui, OpenVR runtime, IPC) into the test binary.
    ${IH_OVERLAY_SRC}/Config.cpp
    # Profile prune tests compile Profiles.cpp directly.
    ${IH_OVERLAY_SRC}/Profiles.cpp
    ${IH_DRIVER_SRC}/InputHealthObservation.cpp
)

target_include_directories(inputhealth_tests
    PRIVATE ${CMAKE_SOURCE_DIR}
    PRIVATE ${IH_OVERLAY_SRC}
    PRIVATE ${IH_DRIVER_SRC}
    PRIVATE ${CMAKE_SOURCE_DIR}/core/src/common
    PRIVATE ${CMAKE_SOURCE_DIR}/core/src/common/inputhealth
    PRIVATE ${CMAKE_SOURCE_DIR}/lib
    PRIVATE ${CMAKE_SOURCE_DIR}/lib/openvr/headers
)

target_link_libraries(inputhealth_tests
    PRIVATE GTest::gtest_main
    PRIVATE openvr_pair_common
)

target_compile_definitions(inputhealth_tests
    PRIVATE NOMINMAX
    PRIVATE UNICODE
    PRIVATE _UNICODE
    PRIVATE INPUTHEALTH_TESTS=1
)

if(MSVC)
    target_compile_options(inputhealth_tests PRIVATE /W4 /wd4100 /wd4505)
endif()

adjust_bin_paths(inputhealth_tests)

include(GoogleTest)
gtest_discover_tests(inputhealth_tests DISCOVERY_TIMEOUT 30)

set_property(TARGET inputhealth_tests PROPERTY FOLDER "tests")
