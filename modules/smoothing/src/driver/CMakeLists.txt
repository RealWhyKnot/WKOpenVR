cmake_minimum_required(VERSION 3.15)

# In the monorepo, the PairDriver root is the repo root (CMAKE_SOURCE_DIR).
set(SMOOTHING_PAIRDRIVER_ROOT "${CMAKE_SOURCE_DIR}")

add_library(openvr_pair_feature_smoothing_driver STATIC
	SkeletalDiagnostics.cpp
	SkeletalDiagnostics.h
	SkeletalHookInjector.cpp
	SkeletalHookInjector.h
	SkeletalSmoothingMath.cpp
	SkeletalSmoothingMath.h
	SmoothingRecorder.cpp
	SmoothingRecorder.h
	SmoothingRecordingSchema.h
	SmoothingDriverModule.cpp)

target_include_directories(openvr_pair_feature_smoothing_driver PRIVATE
	${CMAKE_CURRENT_LIST_DIR}
	${SMOOTHING_PAIRDRIVER_ROOT}/core/src/driver
	${SMOOTHING_PAIRDRIVER_ROOT}/core/src/common
	${SMOOTHING_PAIRDRIVER_ROOT}/lib
	${SMOOTHING_PAIRDRIVER_ROOT}/lib/Eigen
	${SMOOTHING_PAIRDRIVER_ROOT}/lib/openvr/headers
	${SMOOTHING_PAIRDRIVER_ROOT}/lib/minhook/include)

target_compile_definitions(openvr_pair_feature_smoothing_driver PRIVATE
	NOMINMAX
	UNICODE
	OPENVRPAIRDRIVER_EXPORTS)
