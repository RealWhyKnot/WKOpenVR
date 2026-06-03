cmake_minimum_required(VERSION 3.15)

# In the monorepo, the PairDriver root is the repo root (CMAKE_SOURCE_DIR).
set(PHANTOM_PAIRDRIVER_ROOT "${CMAKE_SOURCE_DIR}")

add_library(openvr_pair_feature_phantom_driver STATIC
	# Common headers shared with the overlay + tests.
	${PHANTOM_PAIRDRIVER_ROOT}/modules/phantom/src/common/PhantomStateShmem.cpp
	${PHANTOM_PAIRDRIVER_ROOT}/modules/phantom/src/common/PhantomStateShmem.h
	${PHANTOM_PAIRDRIVER_ROOT}/modules/phantom/src/common/PhantomTypes.h
	${PHANTOM_PAIRDRIVER_ROOT}/modules/phantom/src/common/BlendCurves.h
	${PHANTOM_PAIRDRIVER_ROOT}/modules/phantom/src/common/PoseSample.h
	${PHANTOM_PAIRDRIVER_ROOT}/modules/phantom/src/common/RoleCatalog.h
	# Driver-side math.
	PoseHistory.cpp
	PoseHistory.h
	DeadReckoner.cpp
	DeadReckoner.h
	BodyCompletionSolver.cpp
	BodyCompletionSolver.h
	BlendController.cpp
	BlendController.h
	DropoutState.cpp
	DropoutState.h
	IkFallback.cpp
	IkFallback.h
	VirtualTrackerDevice.cpp
	VirtualTrackerDevice.h
	VirtualTrackerManager.cpp
	VirtualTrackerManager.h
	# Module entry point + IPC dispatch + state ownership. Hot-path entry
	# points are declared in core/src/driver/DriverModule.h next to the
	# CreateDriverModule forward decl so the umbrella driver can call them
	# without taking a per-module include path dependency.
	PhantomDriverModule.cpp)

target_include_directories(openvr_pair_feature_phantom_driver PRIVATE
	${CMAKE_CURRENT_LIST_DIR}
	${PHANTOM_PAIRDRIVER_ROOT}/modules/phantom/src/common
	${PHANTOM_PAIRDRIVER_ROOT}/core/src/driver
	${PHANTOM_PAIRDRIVER_ROOT}/core/src/common
	${PHANTOM_PAIRDRIVER_ROOT}/lib
	${PHANTOM_PAIRDRIVER_ROOT}/lib/openvr/headers)

target_compile_definitions(openvr_pair_feature_phantom_driver PRIVATE
	NOMINMAX
	UNICODE
	OPENVRPAIRDRIVER_EXPORTS)
