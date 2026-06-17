cmake_minimum_required(VERSION 3.15)

add_library(openvr_pair_feature_dynamicres_overlay STATIC
	DynamicResolutionLogic.cpp
	DynamicResolutionLogic.h
	DynamicResolutionPlugin.cpp
	DynamicResolutionPlugin.h
	DynamicResolutionProfile.cpp
	DynamicResolutionProfile.h
	DynamicResolutionStreaming.cpp
	DynamicResolutionStreaming.h)

target_include_directories(openvr_pair_feature_dynamicres_overlay PRIVATE
	${CMAKE_CURRENT_LIST_DIR}
	${CMAKE_SOURCE_DIR}/core/src/common
	${CMAKE_SOURCE_DIR}/core/src/overlay
	${CMAKE_SOURCE_DIR}/lib
	${CMAKE_SOURCE_DIR}/lib/openvr/headers
	${CMAKE_SOURCE_DIR}/lib/imgui)

target_link_libraries(openvr_pair_feature_dynamicres_overlay PRIVATE
	openvr_pair_common
	openvr_pair_overlay_ui
	pair_imgui)

target_compile_definitions(openvr_pair_feature_dynamicres_overlay PRIVATE
	NOMINMAX
	UNICODE
	_UNICODE)

if(MSVC)
	target_compile_options(openvr_pair_feature_dynamicres_overlay PRIVATE
		/W4
		/wd4100
		/wd4505
		/we4715
		/we4828
		/we4473 /we4474 /we4475 /we4476 /we4477 /we4478)
endif()
