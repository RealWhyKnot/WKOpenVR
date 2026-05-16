cmake_minimum_required(VERSION 3.15)

# Phase 3 scaffold: the WKOpenVRPhantomSidecar.exe binary that hosts ML
# pose-completion inference. Ships as a passthrough stub today; the
# follow-up that lands ONNX Runtime + a SparsePoser model replaces the
# inference body without touching the IPC / shmem boundary.

option(OPENVR_PAIR_BUILD_PHANTOM_SIDECAR
    "Build the phantom-tracker inference sidecar exe" ON)

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
    ${PHANTOM_PAIRDRIVER_ROOT}/core/src/common
    ${PHANTOM_PAIRDRIVER_ROOT}/lib/openvr/headers)

set_target_properties(WKOpenVRPhantomSidecar PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/artifacts)

target_compile_definitions(WKOpenVRPhantomSidecar PRIVATE
    NOMINMAX
    UNICODE
    _UNICODE)

# Stage the built exe into the deployable driver tree under
# resources/phantom/host/ so the SidecarSupervisor walk-up resolves it.
add_custom_command(TARGET WKOpenVRPhantomSidecar POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E make_directory
        ${CMAKE_BINARY_DIR}/driver_wkopenvr/resources/phantom/host
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        $<TARGET_FILE:WKOpenVRPhantomSidecar>
        ${CMAKE_BINARY_DIR}/driver_wkopenvr/resources/phantom/host/)
