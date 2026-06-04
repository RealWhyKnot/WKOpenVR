cmake_minimum_required(VERSION 3.15)

# In the monorepo, the PairDriver root is the repo root (CMAKE_SOURCE_DIR).
set(INPUTHEALTH_PAIRDRIVER_ROOT "${CMAKE_SOURCE_DIR}")

add_library(openvr_pair_feature_inputhealth_overlay STATIC
	Config.cpp
	Config.h
	DebugTab.cpp
	DebugTab.h
	DiagnosticsTab.cpp
	DiagnosticsTab.h
	IPCClient.cpp
	IPCClient.h
	InputHealthHealthSummary.cpp
	InputHealthHealthSummary.h
	InputHealthPlugin.cpp
	InputHealthPlugin.h
	LearningEngine.cpp
	LearningEngine.h
	Logging.cpp
	Logging.h
	Profiles.cpp
	Profiles.h
	SettingsTab.cpp
	SettingsTab.h
	SnapshotReader.cpp
	SnapshotReader.h)

target_include_directories(openvr_pair_feature_inputhealth_overlay PRIVATE
	${CMAKE_CURRENT_LIST_DIR}
	${INPUTHEALTH_PAIRDRIVER_ROOT}/core/src/common
	${INPUTHEALTH_PAIRDRIVER_ROOT}/core/src/common/inputhealth
	${INPUTHEALTH_PAIRDRIVER_ROOT}/core/src/overlay
	${CMAKE_SOURCE_DIR}/lib
	${CMAKE_SOURCE_DIR}/lib/imgui
	${CMAKE_SOURCE_DIR}/lib/implot
	${CMAKE_SOURCE_DIR}/lib/openvr/headers)

target_link_libraries(openvr_pair_feature_inputhealth_overlay PRIVATE
	openvr_pair_common)

target_compile_definitions(openvr_pair_feature_inputhealth_overlay PRIVATE
	NOMINMAX
	UNICODE
	_UNICODE
	"OPENVR_INPUTHEALTH_VERSION=\"feature\"")

if(MSVC)
	target_compile_options(openvr_pair_feature_inputhealth_overlay PRIVATE
		/W4
		/wd4100
		/wd4505
		/we4715
		/we4828
		/we4473 /we4474 /we4475 /we4476 /we4477 /we4478)
endif()
