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

add_executable(ui_core_tests
	test_dashboard_input_safe_overlay_logic.cpp
	test_bug_report.cpp
	test_ui_core.cpp
	test_ui_responsive_logic.cpp)

target_include_directories(ui_core_tests PRIVATE
	${CMAKE_SOURCE_DIR}/core/src/common
	${CMAKE_SOURCE_DIR}/core/src/overlay
	${CMAKE_SOURCE_DIR}/lib
	${CMAKE_SOURCE_DIR}/lib/openvr/headers
	${CMAKE_SOURCE_DIR}/lib/imgui)

target_link_libraries(ui_core_tests PRIVATE
	GTest::gtest_main
	openvr_pair_common
	openvr_pair_overlay_ui)

target_compile_definitions(ui_core_tests PRIVATE
	NOMINMAX
	UNICODE
	_UNICODE)

if(MSVC)
	target_compile_options(ui_core_tests PRIVATE /W4 /wd4100)
endif()

adjust_bin_paths(ui_core_tests)

include(GoogleTest)
gtest_discover_tests(ui_core_tests)

set_property(TARGET ui_core_tests PROPERTY FOLDER "tests")
