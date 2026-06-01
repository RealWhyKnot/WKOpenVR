cmake_minimum_required(VERSION 3.14)

# Cross-module e2e tests exercise the facetracking + oscrouter driver libs
# directly. Release builds may skip either module via its disabled-in-release
# marker, in which case the matching driver lib never gets a target and the
# link step would fail with LNK1181. Bail out cleanly when that happens.
if(NOT TARGET openvr_pair_feature_facetracking_driver
		OR NOT TARGET openvr_pair_feature_oscrouter_driver)
	message(STATUS "Skipping e2e_tests (facetracking or oscrouter disabled-in-release)")
	return()
endif()

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

# Cross-module tests launch staged sidecar executables and verify packets at a
# fake VRChat UDP endpoint. Unit tests stay in modules/<name>/tests.
add_executable(e2e_tests
    ${CMAKE_CURRENT_SOURCE_DIR}/e2e_tests.cpp)

target_include_directories(e2e_tests PRIVATE
    ${CMAKE_SOURCE_DIR}/core/src/common
    ${CMAKE_SOURCE_DIR}/core/src/driver
    ${CMAKE_SOURCE_DIR}/modules/facetracking/src/driver
    ${CMAKE_SOURCE_DIR}/modules/oscrouter/src/driver
    ${CMAKE_SOURCE_DIR}/lib
    ${CMAKE_SOURCE_DIR}/lib/openvr/headers)

target_link_libraries(e2e_tests PRIVATE
    GTest::gtest_main
    openvr_pair_common
    openvr_pair_feature_oscrouter_driver
    openvr_pair_feature_facetracking_driver
    Ws2_32)

if(TARGET openvr_pair_feature_facetracking_host)
    add_dependencies(e2e_tests openvr_pair_feature_facetracking_host)

    find_program(E2E_DOTNET_EXECUTABLE
        NAMES dotnet dotnet.exe
        HINTS
            "$ENV{ProgramFiles}/dotnet"
            "$ENV{ProgramFiles\(x86\)}/dotnet")

    if(E2E_DOTNET_EXECUTABLE)
        set(FACE_TEST_MODULE_PROJECT
            "${CMAKE_CURRENT_SOURCE_DIR}/facetracking_test_module/WKOpenVR.FaceTracking.TestModule.csproj")
        set(FACE_TEST_MODULE_ARTIFACT_DIR
            "${CMAKE_BINARY_DIR}/artifacts/FaceTestModule")
        set(FACE_TEST_MODULE_STAMP
            "${CMAKE_CURRENT_BINARY_DIR}/face_test_module.stamp")

        file(GLOB_RECURSE FACE_TEST_MODULE_INPUTS
            "${CMAKE_CURRENT_SOURCE_DIR}/facetracking_test_module/*.csproj"
            "${CMAKE_CURRENT_SOURCE_DIR}/facetracking_test_module/*.cs"
            "${CMAKE_CURRENT_SOURCE_DIR}/facetracking_test_module/*.props"
            "${CMAKE_SOURCE_DIR}/modules/facetracking/src/host/WKOpenVR.FaceTracking.UpstreamSdk/*.csproj"
            "${CMAKE_SOURCE_DIR}/modules/facetracking/src/host/WKOpenVR.FaceTracking.UpstreamSdk/*.cs"
            "${CMAKE_SOURCE_DIR}/modules/facetracking/src/host/WKOpenVR.FaceTracking.UpstreamRuntime/*.csproj"
            "${CMAKE_SOURCE_DIR}/modules/facetracking/src/host/WKOpenVR.FaceTracking.UpstreamRuntime/*.cs")
        list(FILTER FACE_TEST_MODULE_INPUTS EXCLUDE REGEX "/(bin|obj)/")

        add_custom_command(
            OUTPUT "${FACE_TEST_MODULE_STAMP}"
            COMMAND ${CMAKE_COMMAND} -E rm -rf "${FACE_TEST_MODULE_ARTIFACT_DIR}"
            COMMAND ${CMAKE_COMMAND} -E make_directory "${FACE_TEST_MODULE_ARTIFACT_DIR}"
            COMMAND ${E2E_DOTNET_EXECUTABLE} publish "${FACE_TEST_MODULE_PROJECT}"
                -c Release
                --self-contained false
                -p:FaceTestModuleBuildRoot=${CMAKE_BINARY_DIR}/dotnet/e2e-face-test-module/
                -o "${FACE_TEST_MODULE_ARTIFACT_DIR}"
                --nologo
                --verbosity minimal
            COMMAND ${CMAKE_COMMAND} -E touch "${FACE_TEST_MODULE_STAMP}"
            DEPENDS ${FACE_TEST_MODULE_INPUTS}
            WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}/modules/facetracking/src/host"
            COMMENT "Building WKOpenVR e2e face-tracking test module (.NET 10)"
            VERBATIM)

        add_custom_target(openvr_pair_e2e_facetracking_test_module
            DEPENDS "${FACE_TEST_MODULE_STAMP}")
        add_dependencies(openvr_pair_e2e_facetracking_test_module
            openvr_pair_feature_facetracking_host)
        add_dependencies(e2e_tests openvr_pair_e2e_facetracking_test_module)
    else()
        message(WARNING "dotnet CLI not found on PATH. The e2e face-tracking test module will not build.")
    endif()
endif()

if(TARGET WKOpenVRCaptionsHost)
    add_dependencies(e2e_tests WKOpenVRCaptionsHost)
endif()

target_compile_definitions(e2e_tests PRIVATE
    NOMINMAX
    UNICODE
    WKOPENVR_E2E_TESTS=1)

if(MSVC)
    target_compile_options(e2e_tests PRIVATE /bigobj)
endif()

adjust_bin_paths(e2e_tests)

include(GoogleTest)
gtest_discover_tests(e2e_tests)

set_property(TARGET e2e_tests PROPERTY FOLDER "tests")
