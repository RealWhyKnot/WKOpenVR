cmake_minimum_required (VERSION 3.15)

# In the monorepo, SPACECAL_REPO_ROOT is the modules/calibration subtree and
# the PairDriver root is the monorepo root (CMAKE_SOURCE_DIR).
get_filename_component(SPACECAL_REPO_ROOT "${CMAKE_CURRENT_LIST_DIR}/../.." ABSOLUTE)
set(SPACECAL_PAIRDRIVER_ROOT "${CMAKE_SOURCE_DIR}")

if(OPENVR_PAIR_BUILD_UMBRELLA)
    set(SPACECAL_BUILDSTAMP_HEADER "${CMAKE_CURRENT_LIST_DIR}/BuildStamp.h")
    if(NOT EXISTS "${SPACECAL_BUILDSTAMP_HEADER}")
        file(WRITE "${SPACECAL_BUILDSTAMP_HEADER}"
"// Fallback stamp for umbrella cmake builds.\n"
"// Set by WKOpenVR's build.ps1 when running a real release.\n"
"#pragma once\n"
"#define SPACECAL_BUILD_STAMP \"0.0.0.0-DEV\"\n"
"#define SPACECAL_BUILD_CHANNEL \"dev\"\n")
    endif()

    file(GLOB SPACECAL_UMBRELLA_SOURCES CONFIGURE_DEPENDS
        "${CMAKE_CURRENT_LIST_DIR}/*.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/*.h")
    list(FILTER SPACECAL_UMBRELLA_SOURCES EXCLUDE REGEX ".*/SpaceCalibrator\\.cpp$")
    list(FILTER SPACECAL_UMBRELLA_SOURCES EXCLUDE REGEX ".*/SpaceCalibratorPlugin\\.(cpp|h)$")
    list(FILTER SPACECAL_UMBRELLA_SOURCES EXCLUDE REGEX ".*/SpaceCalibratorUmbrellaRuntime\\.(cpp|h)$")
    list(FILTER SPACECAL_UMBRELLA_SOURCES EXCLUDE REGEX ".*/InputHealth.*\\.(cpp|h)$")

    add_library(openvr_pair_feature_calibration_overlay STATIC
        ${SPACECAL_UMBRELLA_SOURCES}
        SpaceCalibratorPlugin.cpp
        SpaceCalibratorPlugin.h
        SpaceCalibratorUmbrellaRuntime.cpp
        SpaceCalibratorUmbrellaRuntime.h)

    target_include_directories(openvr_pair_feature_calibration_overlay PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}
        ${SPACECAL_REPO_ROOT}
        ${SPACECAL_REPO_ROOT}/src/common
        ${SPACECAL_PAIRDRIVER_ROOT}/core/src/common
        ${SPACECAL_PAIRDRIVER_ROOT}/core/src/overlay
        ${CMAKE_SOURCE_DIR}/lib
        ${CMAKE_SOURCE_DIR}/lib/imgui
        ${CMAKE_SOURCE_DIR}/lib/imgui/backends
        ${CMAKE_SOURCE_DIR}/lib/implot
        ${CMAKE_SOURCE_DIR}/lib/openvr/headers
        ${CMAKE_SOURCE_DIR}/lib/Eigen)

    target_link_libraries(openvr_pair_feature_calibration_overlay PUBLIC
        openvr_pair_common
        glfw
        pair_imgui
        pair_implot
        opengl32.lib
        dwmapi.lib
        winhttp.lib
        bcrypt.lib
        ${CMAKE_SOURCE_DIR}/lib/openvr/lib/win64/openvr_api.lib)

    target_compile_definitions(openvr_pair_feature_calibration_overlay PRIVATE
        NOMINMAX
        UNICODE
        _UNICODE
        WIN32_LEAN_AND_MEAN
        EIGEN_MPL2_ONLY
        IMGUI_DEFINE_MATH_OPERATORS)

    if(MSVC)
        target_compile_options(openvr_pair_feature_calibration_overlay PRIVATE /bigobj)
    else()
        target_compile_options(openvr_pair_feature_calibration_overlay PRIVATE -Wa,-mbig-obj)
    endif()

endif()
