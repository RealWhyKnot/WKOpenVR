cmake_minimum_required(VERSION 3.15)

# BuildChannel.h is stamped by build.ps1 on every umbrella invocation and is
# gitignored. When CMake is invoked directly (CI, IDE-driven configure, ad-hoc
# clone) the file is absent and add_library below would error out. Write a
# dev-channel placeholder so configure succeeds; build.ps1 overwrites it with
# the real per-build stamp before MSBuild starts.
set(WKOPENVR_BUILDCHANNEL_HEADER "${CMAKE_CURRENT_LIST_DIR}/BuildChannel.h")
if(NOT EXISTS "${WKOPENVR_BUILDCHANNEL_HEADER}")
	file(WRITE "${WKOPENVR_BUILDCHANNEL_HEADER}"
"// Fallback stamp for umbrella cmake builds.\n"
"// Set by WKOpenVR's build.ps1 when running a real release.\n"
"#pragma once\n"
"#define WKOPENVR_BUILD_STAMP \"0.0.0.0-DEV\"\n"
"#define WKOPENVR_BUILD_CHANNEL \"dev\"\n"
"#define WKOPENVR_BUILD_IS_DEV 1\n")
endif()

add_library(openvr_pair_common STATIC
	BugReport.cpp
	BugReport.h
	BuildChannel.h
	CaptionsRealtimeFlags.h
	CaptionsChatboxPacing.h
	CaptionsSpeechModels.h
	DebugLogging.cpp
	DebugLogging.h
	DiagnosticsLog.cpp
	DiagnosticsLog.h
	FileLog.cpp
	FileLog.h
	DriverMemoryProbe.cpp
	DriverMemoryProbe.h
	HostSupervisorBase.cpp
	HostSupervisorBase.h
	JsonUtil.cpp
	JsonUtil.h
	LogPaths.cpp
	LogPaths.h
	ModuleRegistry.cpp
	ModuleRegistry.h
	ModuleSafety.cpp
	ModuleSafety.h
	ModulePerf.cpp
	ModulePerf.h
	PowerShellCommand.cpp
	PowerShellCommand.h
	RuntimeHealthSummary.cpp
	RuntimeHealthSummary.h
	SidecarOwnerLease.cpp
	SidecarOwnerLease.h
	SteamVrControl.cpp
	SteamVrControl.h
	Win32CommandLine.cpp
	Win32CommandLine.h
	Win32Errors.cpp
	Win32Errors.h
	Win32Paths.cpp
	Win32Paths.h
	Win32Text.cpp
	Win32Text.h)

set_target_properties(openvr_pair_common PROPERTIES
	CXX_STANDARD 20
	CXX_STANDARD_REQUIRED ON
	CXX_EXTENSIONS OFF)

target_include_directories(openvr_pair_common
	PUBLIC
		${CMAKE_CURRENT_LIST_DIR}
		${CMAKE_SOURCE_DIR}/lib)

target_compile_definitions(openvr_pair_common
	PUBLIC
		NOMINMAX)

if(WIN32)
	target_link_libraries(openvr_pair_common
		PUBLIC
			shell32
			ole32
			psapi)
endif()

# Propagate the Eigen include path to every target linking common. The Eigen
# submodule lays headers under lib/Eigen/Eigen/*, so consumers using
# <Eigen/Dense> need lib/Eigen on the include path.
target_link_libraries(openvr_pair_common PUBLIC pair_eigen)
