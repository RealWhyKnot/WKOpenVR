# In-process end-to-end test harness. Built only on dev channel
# (WKOPENVR_RELEASE_BUILD=OFF). The overlay CMakeLists guards
# add_subdirectory(testharness) with the same condition.

cmake_minimum_required(VERSION 3.15)

add_library(wkopenvr_testharness STATIC
	TestHarnessRunner.cpp
	TestHarnessRunner.h
	SteamVRConflictGuard.cpp
	SteamVRConflictGuard.h
	SandboxStaging.cpp
	SandboxStaging.h
	HarnessScenario.cpp
	HarnessScenario.h
	HarnessIpcClient.cpp
	HarnessIpcClient.h
	InProcessDriverLoader.cpp
	InProcessDriverLoader.h
	MockPoseSource.cpp
	MockPoseSource.h
	PhantomReplay.cpp
	PhantomReplay.h
	MockOpenVR/MockDriverContext.cpp
	MockOpenVR/MockDriverContext.h
	MockOpenVR/MockServerDriverHost.cpp
	MockOpenVR/MockServerDriverHost.h
	MockOpenVR/MockDriverInput.cpp
	MockOpenVR/MockDriverInput.h
	MockOpenVR/MockProperties.cpp
	MockOpenVR/MockProperties.h
	MockOpenVR/MockSettings.cpp
	MockOpenVR/MockSettings.h
	Scenario_calibration.cpp
	Scenario_smoothing.cpp
	Scenario_dashboardinput.cpp
	Scenario_inputhealth.cpp
	Scenario_facetracking.cpp
	Scenario_oscrouter.cpp
	Scenario_captions.cpp
	Scenario_phantom.cpp
	Scenario_phantom_replay.cpp)

target_include_directories(wkopenvr_testharness PUBLIC
	${CMAKE_CURRENT_LIST_DIR}/..
	${CMAKE_SOURCE_DIR}/core/src/common
	${CMAKE_SOURCE_DIR}/lib/openvr/headers
	${CMAKE_SOURCE_DIR}/modules/phantom/src/common)

target_link_libraries(wkopenvr_testharness PUBLIC
	openvr_pair_common)

target_compile_definitions(wkopenvr_testharness PRIVATE
	NOMINMAX
	WIN32_LEAN_AND_MEAN
	UNICODE
	_UNICODE)

if(MSVC)
	target_compile_options(wkopenvr_testharness PRIVATE /W3 /permissive-)
endif()
