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

set(PH_DRIVER_SRC "${CMAKE_SOURCE_DIR}/modules/phantom/src/driver")
set(PH_COMMON_SRC "${CMAKE_SOURCE_DIR}/modules/phantom/src/common")

add_executable(phantom_tests
    ${CMAKE_CURRENT_SOURCE_DIR}/test_dead_reckoner.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/test_blend_controller.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/test_dropout_state.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/test_frozen_tracker_invariant.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/test_ik_fallback.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/test_role_catalog.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/test_inference_shmem.cpp
    # Compile the driver-side sources directly so the test binary does not
    # have to link the full driver DLL (which would drag in MinHook,
    # OpenVR runtime, IPC, etc.).
    ${PH_DRIVER_SRC}/PoseHistory.cpp
    ${PH_DRIVER_SRC}/DeadReckoner.cpp
    ${PH_DRIVER_SRC}/BlendController.cpp
    ${PH_DRIVER_SRC}/DropoutState.cpp
    ${PH_DRIVER_SRC}/IkFallback.cpp
    ${PH_COMMON_SRC}/PhantomInferenceShmem.cpp
)

target_include_directories(phantom_tests
    PRIVATE ${PH_DRIVER_SRC}
    PRIVATE ${PH_COMMON_SRC}
    PRIVATE ${CMAKE_SOURCE_DIR}/core/src/common
    PRIVATE ${CMAKE_SOURCE_DIR}/lib
    PRIVATE ${CMAKE_SOURCE_DIR}/lib/openvr/headers
)

target_link_libraries(phantom_tests
    PRIVATE GTest::gtest_main
)

target_compile_definitions(phantom_tests
    PRIVATE NOMINMAX
    PRIVATE UNICODE
    PRIVATE _UNICODE
    PRIVATE PHANTOM_TESTS=1
)

if(MSVC)
    target_compile_options(phantom_tests PRIVATE /W4 /wd4100 /wd4505)
endif()

adjust_bin_paths(phantom_tests)

include(GoogleTest)
gtest_discover_tests(phantom_tests)

set_property(TARGET phantom_tests PROPERTY FOLDER "tests")
