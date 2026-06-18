cmake_minimum_required(VERSION 3.14)

include(FetchContent)
FetchContent_Declare(
	googletest
	URL https://github.com/google/googletest/archive/refs/tags/v1.14.0.zip
)
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(DYNAMICRES_OVERLAY_SRC "${CMAKE_SOURCE_DIR}/modules/dynamicres/src/overlay")

add_executable(dynamicres_tests
	test_dynamic_resolution_logic.cpp
	test_dynamic_resolution_profile.cpp
	${DYNAMICRES_OVERLAY_SRC}/DynamicResolutionLogic.cpp
	${DYNAMICRES_OVERLAY_SRC}/DynamicResolutionProfile.cpp)

target_include_directories(dynamicres_tests PRIVATE
	${DYNAMICRES_OVERLAY_SRC}
	${CMAKE_SOURCE_DIR}/core/src/common
	${CMAKE_SOURCE_DIR}/lib
	${CMAKE_SOURCE_DIR}/lib/openvr/headers)

target_link_libraries(dynamicres_tests PRIVATE
	GTest::gtest_main
	openvr_pair_common)

target_compile_definitions(dynamicres_tests PRIVATE
	NOMINMAX
	UNICODE
	_UNICODE)

if(MSVC)
	target_compile_options(dynamicres_tests PRIVATE /W4 /wd4100 /wd4505)
endif()

adjust_bin_paths(dynamicres_tests)

include(GoogleTest)
gtest_discover_tests(dynamicres_tests)

set_property(TARGET dynamicres_tests PROPERTY FOLDER "tests")
