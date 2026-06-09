cmake_minimum_required(VERSION 3.15)

# In the monorepo, the PairDriver root is the repo root (CMAKE_SOURCE_DIR).
set(FACETRACKING_PAIRDRIVER_ROOT "${CMAKE_SOURCE_DIR}")

# BuildStamp.h is stamped by build.ps1 on every umbrella invocation and is
# gitignored. Same fallback pattern as core/src/common/CMakeLists.txt so a
# direct cmake invocation (CI, IDE-driven configure) can still configure.
set(FACETRACKING_BUILDSTAMP_HEADER "${CMAKE_CURRENT_LIST_DIR}/BuildStamp.h")
if(NOT EXISTS "${FACETRACKING_BUILDSTAMP_HEADER}")
	file(WRITE "${FACETRACKING_BUILDSTAMP_HEADER}"
"// Fallback stamp for umbrella cmake builds.\n"
"// Set by WKOpenVR's build.ps1 when running a real release.\n"
"#pragma once\n"
"#define FACETRACKING_BUILD_STAMP \"0.0.0.0-DEV\"\n"
"#define FACETRACKING_BUILD_CHANNEL \"dev\"\n")
endif()

add_library(openvr_pair_feature_facetracking_overlay STATIC
	AdvancedTab.cpp
	AdvancedTab.h
	AvatarStatePoller.cpp
	AvatarStatePoller.h
	BuildStamp.h
	DriverTelemetryPoller.cpp
	DriverTelemetryPoller.h
	FacetrackingPlugin.cpp
	FacetrackingPlugin.h
	HostStatusPoller.cpp
	HostStatusPoller.h
	IPCClient.cpp
	IPCClient.h
	Logging.cpp
	Logging.h
	LogsSection.cpp
	LogsSection.h
	ModuleSources.cpp
	ModuleSources.h
	ModulesTab.cpp
	ModulesTab.h
	Profiles.cpp
	Profiles.h
	SettingsTab.cpp
	SettingsTab.h
	TuningTab.cpp
	TuningTab.h)

target_include_directories(openvr_pair_feature_facetracking_overlay PRIVATE
	${CMAKE_CURRENT_LIST_DIR}
	${FACETRACKING_PAIRDRIVER_ROOT}/core/src/common
	${FACETRACKING_PAIRDRIVER_ROOT}/core/src/overlay
	${CMAKE_SOURCE_DIR}/lib
	${CMAKE_SOURCE_DIR}/lib/imgui
	${CMAKE_SOURCE_DIR}/lib/implot
	${CMAKE_SOURCE_DIR}/lib/openvr/headers
	${CMAKE_SOURCE_DIR}/lib/picojson)

target_link_libraries(openvr_pair_feature_facetracking_overlay PRIVATE
	openvr_pair_common
	bcrypt
	crypt32)

target_compile_definitions(openvr_pair_feature_facetracking_overlay PRIVATE
	NOMINMAX
	UNICODE
	_UNICODE
	"OPENVR_FACETRACKING_VERSION=\"feature\"")

if(MSVC)
	target_compile_options(openvr_pair_feature_facetracking_overlay PRIVATE
		/W4
		/wd4100
		/wd4505
		/we4715
		/we4828
		/we4473 /we4474 /we4475 /we4476 /we4477 /we4478)
endif()
