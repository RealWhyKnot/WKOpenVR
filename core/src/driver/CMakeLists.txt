cmake_minimum_required(VERSION 3.15)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(TARGET_NAME 01wkopenvr)
set(DRIVER_NAME "driver_wkopenvr")

project(OpenVRPairDriver)
message("WKOpenVR - Driver")

function(GroupSourcesByFolder target)
	set(SOURCE_GROUP_DELIMITER "/")
	set(last_dir "")
	set(files "")
	get_target_property(sources ${target} SOURCES)
	foreach(file ${sources})
		file(RELATIVE_PATH relative_file "${PROJECT_SOURCE_DIR}" ${file})
		get_filename_component(dir "${relative_file}" PATH)
		if(NOT "${dir}" STREQUAL "${last_dir}")
			if(files)
				source_group("${last_dir}" FILES ${files})
			endif()
			set(files "")
		endif()
		set(files ${files} ${file})
		set(last_dir "${dir}")
	endforeach()
	if(files)
		source_group("${last_dir}" FILES ${files})
	endif()
endfunction()

set(OPENVR_LIB_DIR ${CMAKE_SOURCE_DIR}/lib/openvr)

if(NOT PLATFORM)
	if(CMAKE_SIZEOF_VOID_P MATCHES 8)
		set(PLATFORM 64)
	else()
		set(PLATFORM 32)
	endif()
endif()

if(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
	add_definitions(-D_WIN32)
	set(ARCH_TARGET win${PLATFORM})
	set(WINDOWS_PATH_SUFFIXES win${PLATFORM} Win${PLATFORM} x${PLATFORM})
endif()

find_library(OPENVR_LIBRARIES
	NAMES openvr_api
	PATHS ${OPENVR_LIB_DIR}/bin ${OPENVR_LIB_DIR}/lib
	PATH_SUFFIXES win${PLATFORM}
	NO_DEFAULT_PATH NO_CMAKE_FIND_ROOT_PATH)

set(OPENVR_PAIRDRIVER_ROOT ${CMAKE_SOURCE_DIR} CACHE PATH "WKOpenVR checkout" FORCE)
set(OPENVR_PAIR_FEATURE_LIBS "")
set(OPENVR_PAIR_FEATURE_DEFINES "")

# Feature modules are always present in the monorepo; add unconditionally.
add_subdirectory("${CMAKE_SOURCE_DIR}/modules/inputhealth/src/driver" "${CMAKE_BINARY_DIR}/modules/inputhealth-driver")
list(APPEND OPENVR_PAIR_FEATURE_LIBS openvr_pair_feature_inputhealth_driver)
list(APPEND OPENVR_PAIR_FEATURE_DEFINES OPENVR_PAIR_HAS_INPUTHEALTH_DRIVER=1)

add_subdirectory("${CMAKE_SOURCE_DIR}/modules/smoothing/src/driver" "${CMAKE_BINARY_DIR}/modules/smoothing-driver")
list(APPEND OPENVR_PAIR_FEATURE_LIBS openvr_pair_feature_smoothing_driver)
list(APPEND OPENVR_PAIR_FEATURE_DEFINES OPENVR_PAIR_HAS_SMOOTHING_DRIVER=1)

add_subdirectory("${CMAKE_SOURCE_DIR}/modules/calibration/src/driver" "${CMAKE_BINARY_DIR}/modules/calibration-driver")
list(APPEND OPENVR_PAIR_FEATURE_LIBS openvr_pair_feature_calibration_driver)
list(APPEND OPENVR_PAIR_FEATURE_DEFINES OPENVR_PAIR_HAS_CALIBRATION_DRIVER=1)

add_subdirectory("${CMAKE_SOURCE_DIR}/modules/facetracking/src/driver" "${CMAKE_BINARY_DIR}/modules/facetracking-driver")
list(APPEND OPENVR_PAIR_FEATURE_LIBS openvr_pair_feature_facetracking_driver)
list(APPEND OPENVR_PAIR_FEATURE_DEFINES OPENVR_PAIR_HAS_FACETRACKING_DRIVER=1)

add_subdirectory("${CMAKE_SOURCE_DIR}/modules/oscrouter/src/driver" "${CMAKE_BINARY_DIR}/modules/oscrouter-driver")
list(APPEND OPENVR_PAIR_FEATURE_LIBS openvr_pair_feature_oscrouter_driver)
list(APPEND OPENVR_PAIR_FEATURE_DEFINES OPENVR_PAIR_HAS_OSCROUTER_DRIVER=1)

add_subdirectory("${CMAKE_SOURCE_DIR}/modules/captions/src/driver" "${CMAKE_BINARY_DIR}/modules/captions-driver")
list(APPEND OPENVR_PAIR_FEATURE_LIBS openvr_pair_feature_captions_driver)
list(APPEND OPENVR_PAIR_FEATURE_DEFINES OPENVR_PAIR_HAS_CAPTIONS_DRIVER=1)

add_subdirectory("${CMAKE_SOURCE_DIR}/modules/phantom/src/driver" "${CMAKE_BINARY_DIR}/modules/phantom-driver")
list(APPEND OPENVR_PAIR_FEATURE_LIBS openvr_pair_feature_phantom_driver)
list(APPEND OPENVR_PAIR_FEATURE_DEFINES OPENVR_PAIR_HAS_PHANTOM_DRIVER=1)

file(GLOB_RECURSE SOURCES_DRIVER ${CMAKE_SOURCE_DIR}/core/src/driver "*.c" "*.h" "*.hpp" "*.cpp")
file(GLOB_RECURSE SOURCES_COMMON ${CMAKE_SOURCE_DIR}/core/src/common "*.h" "*.hpp")

foreach(SOURCE IN ITEMS ${SOURCES_DRIVER})
	get_filename_component(SOURCE_PATH "${SOURCE}" PATH)
	string(REPLACE "${CMAKE_CURRENT_SOURCE_DIR}" "" GROUP_PATH "${SOURCE_PATH}")
	string(REPLACE "/" "\\" GROUP_PATH "${GROUP_PATH}")
	source_group("${GROUP_PATH}" FILES "${SOURCE}")
endforeach()

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY $<1:${CMAKE_BINARY_DIR}/${DRIVER_NAME}/bin/${ARCH_TARGET}>)

add_library(OpenVRPairDriver SHARED ${SOURCES_DRIVER} ${SOURCES_COMMON})
GroupSourcesByFolder(OpenVRPairDriver)

target_include_directories(OpenVRPairDriver
	PRIVATE ${CMAKE_SOURCE_DIR}
	PRIVATE ${CMAKE_SOURCE_DIR}/core/src/common
	PRIVATE ${CMAKE_SOURCE_DIR}/core/src/driver
	PRIVATE ${OPENVR_LIB_DIR}
	PRIVATE ${CMAKE_SOURCE_DIR}/lib/openvr/headers
	PRIVATE ${CMAKE_SOURCE_DIR}/lib/minhook/include
	PRIVATE ${CMAKE_SOURCE_DIR}/lib)

target_link_libraries(OpenVRPairDriver
	PRIVATE openvr_pair_common
	PRIVATE ${OPENVR_LIBRARIES}
	PRIVATE minhook
	PRIVATE ${OPENVR_PAIR_FEATURE_LIBS})

# Pull the build version stamp from version.txt at the repo root and inject
# it as PAIRDRIVER_VERSION_STRING. build.ps1 writes this file for every build
# (release CI passes the tag, local builds derive YYYY.M.D.N-XXXX).
file(READ "${CMAKE_SOURCE_DIR}/version.txt" PAIRDRIVER_VERSION_RAW)
string(STRIP "${PAIRDRIVER_VERSION_RAW}" PAIRDRIVER_VERSION_STAMP)

target_compile_definitions(OpenVRPairDriver
	PRIVATE NOMINMAX
	PRIVATE UNICODE
	PRIVATE OPENVRPAIRDRIVER_EXPORTS
	PRIVATE ${OPENVR_PAIR_FEATURE_DEFINES}
	PRIVATE "PAIRDRIVER_VERSION_STRING=\"${PAIRDRIVER_VERSION_STAMP}\"")

set_target_properties(OpenVRPairDriver PROPERTIES OUTPUT_NAME ${DRIVER_NAME})

# Stage the driver folder (manifest + resources) next to the built DLL so
# `<build>/driver_wkopenvr/` is a complete deployable tree the consumer
# repos can copy to <SteamVR>\drivers\.
add_custom_command(
	TARGET OpenVRPairDriver
	PRE_BUILD
	COMMAND ${CMAKE_COMMAND} -E copy_directory
	${CMAKE_SOURCE_DIR}/${DRIVER_NAME}
	${CMAKE_BINARY_DIR}/${DRIVER_NAME})

# Stage the phantom module's vive_tracker_<role>_profile.json files into
# the deployable driver tree's resources/input/ directory. Phase 2's
# VirtualTrackerDevice references these via Prop_InputProfilePath_String
# with the "{wkopenvr}/input/<file>" macro so SteamVR resolves them
# under <SteamVR>\drivers\01wkopenvr\resources\input\.
file(GLOB PHANTOM_INPUT_PROFILES
	"${CMAKE_SOURCE_DIR}/modules/phantom/resources/input/vive_tracker_*_profile.json")
add_custom_command(
	TARGET OpenVRPairDriver
	POST_BUILD
	COMMAND ${CMAKE_COMMAND} -E make_directory
		${CMAKE_BINARY_DIR}/${DRIVER_NAME}/resources/input)
foreach(_profile ${PHANTOM_INPUT_PROFILES})
	add_custom_command(
		TARGET OpenVRPairDriver
		POST_BUILD
		COMMAND ${CMAKE_COMMAND} -E copy_if_different
			"${_profile}"
			"${CMAKE_BINARY_DIR}/${DRIVER_NAME}/resources/input/")
endforeach()
