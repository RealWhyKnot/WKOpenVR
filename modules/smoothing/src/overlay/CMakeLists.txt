cmake_minimum_required(VERSION 3.15)

# In the monorepo, the PairDriver root is the repo root (CMAKE_SOURCE_DIR).
set(SMOOTHING_PAIRDRIVER_ROOT "${CMAKE_SOURCE_DIR}")

add_library(openvr_pair_feature_smoothing_overlay STATIC
	Config.cpp
	Config.h
	IPCClient.cpp
	IPCClient.h
	Logging.cpp
	Logging.h
	SmoothingFingers.cpp
	SmoothingPlugin.cpp
	SmoothingPlugin.h
	SmoothingPrediction.cpp
	SmoothingPredictionLogic.h)

target_include_directories(openvr_pair_feature_smoothing_overlay PRIVATE
	${CMAKE_CURRENT_LIST_DIR}
	${SMOOTHING_PAIRDRIVER_ROOT}/core/src/common
	${SMOOTHING_PAIRDRIVER_ROOT}/core/src/overlay
	${CMAKE_SOURCE_DIR}/lib/imgui
	${CMAKE_SOURCE_DIR}/lib/openvr/headers)

target_link_libraries(openvr_pair_feature_smoothing_overlay PRIVATE
	openvr_pair_common)

target_compile_definitions(openvr_pair_feature_smoothing_overlay PRIVATE
	NOMINMAX
	UNICODE
	_UNICODE)
