cmake_minimum_required(VERSION 3.14)

include(FetchContent)
if(NOT TARGET GTest::gtest_main)
	FetchContent_Declare(
		googletest
		URL https://github.com/google/googletest/archive/refs/tags/v1.14.0.zip
	)
	set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
	FetchContent_MakeAvailable(googletest)
endif()

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_executable(driver_safety_tests
	test_module_registry.cpp
	test_module_safety.cpp
	test_safe_mode_recovery.cpp
	test_sidecar_owner_lease.cpp)

target_include_directories(driver_safety_tests PRIVATE
	${CMAKE_SOURCE_DIR}/core/src/common
	${CMAKE_SOURCE_DIR}/lib)

target_link_libraries(driver_safety_tests PRIVATE
	GTest::gtest_main
	openvr_pair_common)

target_compile_definitions(driver_safety_tests PRIVATE
	NOMINMAX
	UNICODE
	_UNICODE)

if(MSVC)
	target_compile_options(driver_safety_tests PRIVATE /W4 /wd4100)
endif()

adjust_bin_paths(driver_safety_tests)

include(GoogleTest)
gtest_discover_tests(driver_safety_tests)

set_property(TARGET driver_safety_tests PROPERTY FOLDER "tests")
