cmake_minimum_required(VERSION 3.15)

set(DASHBOARDINPUT_PAIRDRIVER_ROOT "${CMAKE_SOURCE_DIR}")

add_library(openvr_pair_feature_dashboardinput_driver STATIC
	DashboardInputDriverModule.cpp)

target_include_directories(openvr_pair_feature_dashboardinput_driver PRIVATE
	${CMAKE_CURRENT_LIST_DIR}
	${DASHBOARDINPUT_PAIRDRIVER_ROOT}/core/src/driver
	${DASHBOARDINPUT_PAIRDRIVER_ROOT}/core/src/common
	${DASHBOARDINPUT_PAIRDRIVER_ROOT}/modules/smoothing/src/driver
	${DASHBOARDINPUT_PAIRDRIVER_ROOT}/lib
	${DASHBOARDINPUT_PAIRDRIVER_ROOT}/lib/Eigen
	${DASHBOARDINPUT_PAIRDRIVER_ROOT}/lib/openvr/headers
	${DASHBOARDINPUT_PAIRDRIVER_ROOT}/lib/minhook/include)

target_link_libraries(openvr_pair_feature_dashboardinput_driver PRIVATE
	openvr_pair_feature_smoothing_driver)

target_compile_definitions(openvr_pair_feature_dashboardinput_driver PRIVATE
	NOMINMAX
	UNICODE
	OPENVRPAIRDRIVER_EXPORTS)
