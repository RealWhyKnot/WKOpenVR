cmake_minimum_required(VERSION 3.15)

set(PHANTOM_PAIRDRIVER_ROOT "${CMAKE_SOURCE_DIR}")

add_library(openvr_pair_feature_phantom_overlay STATIC
	# Common headers shared with the driver.
	${PHANTOM_PAIRDRIVER_ROOT}/modules/phantom/src/common/PhantomStateShmem.cpp
	${PHANTOM_PAIRDRIVER_ROOT}/modules/phantom/src/common/PhantomStateShmem.h
	${PHANTOM_PAIRDRIVER_ROOT}/modules/phantom/src/common/PhantomTypes.h
	${PHANTOM_PAIRDRIVER_ROOT}/modules/phantom/src/common/BlendCurves.h
	${PHANTOM_PAIRDRIVER_ROOT}/modules/phantom/src/common/RoleCatalog.h
	# Overlay-side.
	Config.cpp
	Config.h
	IPCClient.cpp
	IPCClient.h
	PhantomPlugin.cpp
	PhantomPlugin.h
	PhantomUiLogic.h)

target_include_directories(openvr_pair_feature_phantom_overlay PRIVATE
	${CMAKE_CURRENT_LIST_DIR}
	${PHANTOM_PAIRDRIVER_ROOT}/modules/phantom/src/common
	${PHANTOM_PAIRDRIVER_ROOT}/core/src/common
	${PHANTOM_PAIRDRIVER_ROOT}/core/src/overlay
	${CMAKE_SOURCE_DIR}/lib/imgui
	${CMAKE_SOURCE_DIR}/lib/openvr/headers)

target_link_libraries(openvr_pair_feature_phantom_overlay PRIVATE
	openvr_pair_common)

target_compile_definitions(openvr_pair_feature_phantom_overlay PRIVATE
	NOMINMAX
	UNICODE
	_UNICODE)
