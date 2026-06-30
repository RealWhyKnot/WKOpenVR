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
set(PH_OVERLAY_SRC "${CMAKE_SOURCE_DIR}/modules/phantom/src/overlay")
set(PH_ANALYSIS_SRC "${CMAKE_SOURCE_DIR}/modules/phantom/src/analysis")

add_executable(phantom_tests
    ${CMAKE_CURRENT_SOURCE_DIR}/test_dead_reckoner.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/test_blend_controller.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/test_dropout_state.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/test_body_prior_estimator.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/test_body_completion_solver.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/test_body_completion_scenarios.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/test_dropout_scenarios.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/test_frozen_tracker_invariant.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/test_ik_fallback.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/test_role_catalog.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/test_inference_shmem.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/test_passive_role_inference.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/test_snap_calibrate.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/test_role_arbiter.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/test_phantom_replay_player.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/test_phantom_ui_logic.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/test_phantom_metrics.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/test_phantom_trajectory.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/test_virtual_tracker_lifecycle.cpp
    # Compile the driver-side sources directly so the test binary does not
    # have to link the full driver DLL (which would drag in MinHook,
    # OpenVR runtime, IPC, etc.). VirtualTracker* use a header-only fake driver
    # context (FakeDriverHost.h) plus teststubs/ for Logging / DebugLogging.
    ${PH_DRIVER_SRC}/PoseHistory.cpp
    ${PH_DRIVER_SRC}/DeadReckoner.cpp
    ${PH_DRIVER_SRC}/BlendController.cpp
    ${PH_DRIVER_SRC}/BodyCompletionSolver.cpp
    ${PH_DRIVER_SRC}/DropoutState.cpp
    ${PH_DRIVER_SRC}/IkFallback.cpp
    ${PH_DRIVER_SRC}/VirtualTrackerDevice.cpp
    ${PH_DRIVER_SRC}/VirtualTrackerManager.cpp
    ${PH_COMMON_SRC}/PhantomInferenceShmem.cpp
)

target_include_directories(phantom_tests
    # teststubs first: shadow the driver Logging.h / common DebugLogging.h whose
    # backing .cpp files the unit-test target does not link.
    PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/teststubs
    PRIVATE ${PH_DRIVER_SRC}
    PRIVATE ${PH_COMMON_SRC}
    PRIVATE ${PH_OVERLAY_SRC}
    PRIVATE ${PH_ANALYSIS_SRC}
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
