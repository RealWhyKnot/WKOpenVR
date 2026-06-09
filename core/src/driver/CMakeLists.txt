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

# Feature modules are gated through cmake/WKOpenVRModules.cmake: each helper
# call adds the module's driver subdir, registers its lib for linking, and
# emits its OPENVR_PAIR_HAS_*_DRIVER define -- unless the module ships a
# modules/<slug>/disabled-in-release.flag marker AND the build was configured
# with -DWKOPENVR_RELEASE_BUILD=ON, in which case it's skipped silently.
wkopenvr_add_driver_module(inputhealth)
wkopenvr_add_driver_module(smoothing)
wkopenvr_add_driver_module(dashboardinput)
wkopenvr_add_driver_module(calibration)
wkopenvr_add_driver_module(facetracking)
wkopenvr_add_driver_module(oscrouter)
wkopenvr_add_driver_module(captions)
wkopenvr_add_driver_module(phantom)

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

target_compile_definitions(OpenVRPairDriver
	PRIVATE NOMINMAX
	PRIVATE UNICODE
	PRIVATE OPENVRPAIRDRIVER_EXPORTS
	PRIVATE ${OPENVR_PAIR_FEATURE_DEFINES}
	PRIVATE "PAIRDRIVER_VERSION_STRING=\"${WKOPENVR_VERSION_STAMP}\"")

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
# with the "{01wkopenvr}/input/<file>" macro so SteamVR resolves them
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
