cmake_minimum_required(VERSION 3.15)

# Optional future Phantom backend. The driver does not launch this binary
# in the deterministic completion path; it remains buildable so a later
# backend can reuse the existing process and shared-memory boundary.

# Belt-and-suspenders guard for the disabled-in-release path (see the
# matching note in modules/facetracking/src/host/CMakeLists.txt).
if(COMMAND wkopenvr_module_disabled)
    wkopenvr_module_disabled(phantom _wk_ph_disabled)
    if(_wk_ph_disabled)
        message(STATUS "Skipping phantom host module (disabled-in-release; secondary guard)")
        return()
    endif()
endif()

option(OPENVR_PAIR_BUILD_PHANTOM_SIDECAR
    "Build the phantom-tracker optional backend exe" ON)

if(NOT OPENVR_PAIR_BUILD_PHANTOM_SIDECAR)
    message(STATUS "Phantom sidecar build disabled")
    return()
endif()

set(PHANTOM_PAIRDRIVER_ROOT "${CMAKE_SOURCE_DIR}")

add_executable(WKOpenVRPhantomSidecar
    Sidecar.cpp
    ${PHANTOM_PAIRDRIVER_ROOT}/modules/phantom/src/common/PhantomInferenceShmem.cpp
    ${PHANTOM_PAIRDRIVER_ROOT}/modules/phantom/src/common/PhantomInferenceShmem.h
    ${PHANTOM_PAIRDRIVER_ROOT}/modules/phantom/src/common/PhantomTypes.h
    ${PHANTOM_PAIRDRIVER_ROOT}/modules/phantom/src/common/RoleCatalog.h)

target_include_directories(WKOpenVRPhantomSidecar PRIVATE
    ${PHANTOM_PAIRDRIVER_ROOT}/modules/phantom/src/common
    ${PHANTOM_PAIRDRIVER_ROOT}/modules/phantom/src/driver
    ${PHANTOM_PAIRDRIVER_ROOT}/modules/phantom/src/analysis
    ${PHANTOM_PAIRDRIVER_ROOT}/core/src/common
    ${PHANTOM_PAIRDRIVER_ROOT}/lib/openvr/headers)

target_compile_features(WKOpenVRPhantomSidecar PRIVATE cxx_std_20)

set_target_properties(WKOpenVRPhantomSidecar PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/artifacts)

target_compile_definitions(WKOpenVRPhantomSidecar PRIVATE
    NOMINMAX
    UNICODE
    _UNICODE)

# Stage the built exe into the deployable driver tree under
# resources/phantom/host/ for optional backend development.
add_custom_command(TARGET WKOpenVRPhantomSidecar POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E make_directory
        ${CMAKE_BINARY_DIR}/driver_wkopenvr/resources/phantom/host
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        $<TARGET_FILE:WKOpenVRPhantomSidecar>
        ${CMAKE_BINARY_DIR}/driver_wkopenvr/resources/phantom/host/)
