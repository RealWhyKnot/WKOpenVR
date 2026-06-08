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

set(QUESTAPP_OVERLAY_SRC "${CMAKE_SOURCE_DIR}/modules/questapp/src/overlay")

add_executable(questapp_tests
    ${CMAKE_CURRENT_SOURCE_DIR}/test_adb_controller.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/test_adb_setup_wizard.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/test_quest_companion_protocol.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/test_questapp_catalog.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/test_questapp_config.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/test_questapp_mic_recovery.cpp
    ${QUESTAPP_OVERLAY_SRC}/AdbController.cpp
    ${QUESTAPP_OVERLAY_SRC}/AdbSetupWizard.cpp
    ${QUESTAPP_OVERLAY_SRC}/QuestAppCatalog.cpp
    ${QUESTAPP_OVERLAY_SRC}/QuestCompanionProtocol.cpp
    ${QUESTAPP_OVERLAY_SRC}/QuestAppConfig.cpp
    ${QUESTAPP_OVERLAY_SRC}/QuestAppMicRecovery.cpp)

target_include_directories(questapp_tests
    PRIVATE ${QUESTAPP_OVERLAY_SRC}
    PRIVATE ${CMAKE_SOURCE_DIR}/core/src/common
    PRIVATE ${CMAKE_SOURCE_DIR}/core/src/overlay
    PRIVATE ${CMAKE_SOURCE_DIR}/lib)

target_link_libraries(questapp_tests
    PRIVATE GTest::gtest_main
    PRIVATE openvr_pair_common
    PRIVATE bcrypt
    PRIVATE shell32)

target_compile_definitions(questapp_tests
    PRIVATE NOMINMAX
    PRIVATE UNICODE
    PRIVATE _UNICODE)

if(MSVC)
    target_compile_options(questapp_tests PRIVATE /W4 /wd4100 /wd4505)
endif()

adjust_bin_paths(questapp_tests)

include(GoogleTest)
gtest_discover_tests(questapp_tests)

set_property(TARGET questapp_tests PROPERTY FOLDER "tests")
