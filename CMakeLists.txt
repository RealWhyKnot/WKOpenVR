cmake_minimum_required(VERSION 3.15)

set_property(GLOBAL PROPERTY USE_FOLDERS ON)

project("OpenVR-WKPair")

# ---------------------------------------------------------------------------
# Compiler cache (sccache preferred, ccache fallback).
# Set CMAKE_C/CXX_COMPILER_LAUNCHER so every C/C++ target benefits.
# Neither tool changes build outputs; if absent the build is unaffected.
# ---------------------------------------------------------------------------
find_program(SCCACHE_BIN sccache)
find_program(CCACHE_BIN  ccache)
if(SCCACHE_BIN)
    set(WKOPENVR_COMPILER_LAUNCHER "${SCCACHE_BIN}")
    set(WKOPENVR_COMPILER_LAUNCHER_DISPLAY "sccache (${SCCACHE_BIN})")
elseif(CCACHE_BIN)
    set(WKOPENVR_COMPILER_LAUNCHER "${CCACHE_BIN}")
    set(WKOPENVR_COMPILER_LAUNCHER_DISPLAY "ccache (${CCACHE_BIN})")
else()
    set(WKOPENVR_COMPILER_LAUNCHER "")
    set(WKOPENVR_COMPILER_LAUNCHER_DISPLAY "none -- install sccache or ccache to speed up rebuilds")
endif()
if(WKOPENVR_COMPILER_LAUNCHER)
    set(CMAKE_C_COMPILER_LAUNCHER   "${WKOPENVR_COMPILER_LAUNCHER}" CACHE STRING "" FORCE)
    set(CMAKE_CXX_COMPILER_LAUNCHER "${WKOPENVR_COMPILER_LAUNCHER}" CACHE STRING "" FORCE)
endif()

# Build artifacts go under build/artifacts so we can find them from build.ps1
# and the consumer repos' deploy scripts without grepping.
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/artifacts)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/artifacts)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/artifacts)

# adjust_bin_paths: redirect a target's output directories to artifacts/.
# Used by module CMakeLists (e.g. spacecal_tests) that were originally built
# standalone and call this helper.
function(adjust_bin_paths target_name)
    set_target_properties(${target_name} PROPERTIES
        RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/artifacts
        LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/artifacts
        ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/artifacts)
endfunction()

# Release-only LTCG. The driver isn't math-heavy enough for it to matter much,
# but the InterfaceHookInjector + SkeletalHookInjector detours are on hot paths
# (340 Hz/hand for skeletal, kHz across all tracked devices for pose) and any
# inlining win there is worth the linker time.
set(CMAKE_INTERPROCEDURAL_OPTIMIZATION_RELEASE ON)

# lib first: third-party targets must exist before the feature modules reference them.
add_subdirectory(lib)

# Core umbrella driver + overlay. These CMakeLists add the feature
# module driver/overlay subdirectories directly.
add_subdirectory(core/src)

# Face-tracking C# host sidecar. Built via `dotnet publish` and staged into the
# artifacts tree by build.ps1. Toggleable via OPENVR_PAIR_BUILD_FACE_HOST so a
# C++-only contributor doesn't need the .NET SDK to build the driver+overlay.
add_subdirectory(modules/facetracking/src/host)

# Captions native C++ host sidecar. Toggleable via OPENVR_PAIR_BUILD_CAPTIONS_HOST.
add_subdirectory(modules/captions/src/host)

# Phantom inference sidecar (Phase 3 scaffold). Toggleable via
# OPENVR_PAIR_BUILD_PHANTOM_SIDECAR; off lets contributors who only touch
# the driver/overlay skip the sidecar build.
add_subdirectory(modules/phantom/src/host)

# Test suites. Unit tests stay in their owning module folders; cross-module E2E
# tests live under tests/e2e and are registered alongside the rest.
add_subdirectory(tests)

message(STATUS "------ WKOpenVR build configuration ------")
message(STATUS "  Compiler cache:   ${WKOPENVR_COMPILER_LAUNCHER_DISPLAY}")
message(STATUS "  Captions CUDA:    ${WKOPENVR_CAPTIONS_CUDA}")
message(STATUS "  Captions host:    ${OPENVR_PAIR_BUILD_CAPTIONS_HOST}")
message(STATUS "------------------------------------------")

if("${CMAKE_CXX_COMPILER_ID}" STREQUAL "MSVC" OR "${CMAKE_C_COMPILER_ID}" STREQUAL "MSVC")
	add_compile_options(
		/we4715  # not all control paths return a value
		/we4828  # disallow invalid characters
		/we4473 /we4474 /we4475 /we4476 /we4477 /we4478  # printf format-string mismatches
		/we4775 /we4776 /we4777 /we4778
		/we4002 /we4003  # macro arg count mismatches
		/MP      # multiprocessor compilation
		/utf-8
		/GF)     # fold duplicate strings
	add_compile_options(
		$<$<CONFIG:Release>:/fp:fast>
		$<$<CONFIG:Release>:/arch:AVX2>)
endif()
