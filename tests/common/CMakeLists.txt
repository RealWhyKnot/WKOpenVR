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

add_executable(common_helpers_tests
	test_powershell_command.cpp
	test_win32_command_line.cpp)

target_include_directories(common_helpers_tests PRIVATE
	${CMAKE_SOURCE_DIR}/core/src/common
	${CMAKE_SOURCE_DIR}/lib)

target_link_libraries(common_helpers_tests PRIVATE
	GTest::gtest_main
	openvr_pair_common)

target_compile_definitions(common_helpers_tests PRIVATE
	NOMINMAX
	UNICODE
	_UNICODE)

if(MSVC)
	target_compile_options(common_helpers_tests PRIVATE /W4 /wd4100)
endif()

adjust_bin_paths(common_helpers_tests)

include(GoogleTest)
gtest_discover_tests(common_helpers_tests)

set_property(TARGET common_helpers_tests PROPERTY FOLDER "tests")
