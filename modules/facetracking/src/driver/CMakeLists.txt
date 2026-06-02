cmake_minimum_required(VERSION 3.15)

# In the monorepo, the PairDriver root is the repo root (CMAKE_SOURCE_DIR).
set(FACETRACKING_PAIRDRIVER_ROOT "${CMAKE_SOURCE_DIR}")

add_library(openvr_pair_feature_facetracking_driver STATIC
	CalibrationEngine.cpp
	CalibrationEngine.h
	EyelidSync.cpp
	EyelidSync.h
	FaceFrameReader.cpp
	FaceFrameReader.h
	FaceOscAddressFilter.cpp
	FaceOscPublisher.cpp
	FaceOscPublisher.h
	FaceSignalProcessor.cpp
	FaceSignalProcessor.h
	FaceTrackingDevice.cpp
	FaceTrackingDevice.h
	FacetrackingDriverModule.cpp
	HostSupervisor.cpp
	HostSupervisor.h
	Logging.cpp
	Logging.h
	VergenceLock.cpp
	VergenceLock.h)

target_include_directories(openvr_pair_feature_facetracking_driver PRIVATE
	${CMAKE_CURRENT_LIST_DIR}
	${FACETRACKING_PAIRDRIVER_ROOT}/core/src/driver
	${FACETRACKING_PAIRDRIVER_ROOT}/core/src/common
	${FACETRACKING_PAIRDRIVER_ROOT}/lib
	${FACETRACKING_PAIRDRIVER_ROOT}/lib/Eigen
	${FACETRACKING_PAIRDRIVER_ROOT}/lib/openvr/headers
	${FACETRACKING_PAIRDRIVER_ROOT}/lib/picojson
	${FACETRACKING_PAIRDRIVER_ROOT}/modules/oscrouter/src/driver)

target_link_libraries(openvr_pair_feature_facetracking_driver PRIVATE
	openvr_pair_common
	openvr_pair_feature_oscrouter_driver)

target_compile_definitions(openvr_pair_feature_facetracking_driver PRIVATE
	NOMINMAX
	UNICODE
	OPENVRPAIRDRIVER_EXPORTS)
