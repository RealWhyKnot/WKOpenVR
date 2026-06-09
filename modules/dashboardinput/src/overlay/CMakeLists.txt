cmake_minimum_required(VERSION 3.15)

set(DASHBOARDINPUT_PAIRDRIVER_ROOT "${CMAKE_SOURCE_DIR}")

add_library(openvr_pair_feature_dashboardinput_overlay STATIC
	DashboardInputPlugin.cpp
	DashboardInputPlugin.h
	IPCClient.cpp
	IPCClient.h)

target_include_directories(openvr_pair_feature_dashboardinput_overlay PRIVATE
	${CMAKE_CURRENT_LIST_DIR}
	${DASHBOARDINPUT_PAIRDRIVER_ROOT}/core/src/common
	${DASHBOARDINPUT_PAIRDRIVER_ROOT}/core/src/overlay
	${CMAKE_SOURCE_DIR}/lib/imgui
	${CMAKE_SOURCE_DIR}/lib/openvr/headers)

target_link_libraries(openvr_pair_feature_dashboardinput_overlay PRIVATE
	openvr_pair_common
	openvr_pair_overlay_ui)

target_compile_definitions(openvr_pair_feature_dashboardinput_overlay PRIVATE
	NOMINMAX
	UNICODE
	_UNICODE)
