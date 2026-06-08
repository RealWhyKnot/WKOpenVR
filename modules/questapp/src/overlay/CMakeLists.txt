cmake_minimum_required(VERSION 3.15)

add_library(openvr_pair_feature_questapp_overlay STATIC
    AdbController.cpp
    AdbController.h
    AdbSetupWizard.cpp
    AdbSetupWizard.h
    QuestAppCatalog.cpp
    QuestAppCatalog.h
    QuestAppConfig.cpp
    QuestAppConfig.h
    QuestCompanionProtocol.cpp
    QuestCompanionProtocol.h
    QuestAppInstaller.cpp
    QuestAppInstaller.h
    QuestAppMicRecovery.cpp
    QuestAppMicRecovery.h
    QuestAppPlugin.cpp
    QuestAppPlugin.h)

target_include_directories(openvr_pair_feature_questapp_overlay PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}
    ${CMAKE_SOURCE_DIR}/core/src/common
    ${CMAKE_SOURCE_DIR}/core/src/overlay
    ${CMAKE_SOURCE_DIR}/lib
    ${CMAKE_SOURCE_DIR}/lib/imgui)

target_link_libraries(openvr_pair_feature_questapp_overlay PRIVATE
    openvr_pair_common
    openvr_pair_overlay_ui
    pair_imgui)

target_link_libraries(openvr_pair_feature_questapp_overlay PUBLIC
    bcrypt
    shell32
    winhttp)

target_compile_definitions(openvr_pair_feature_questapp_overlay PRIVATE
    NOMINMAX
    UNICODE
    _UNICODE)

if(MSVC)
    target_compile_options(openvr_pair_feature_questapp_overlay PRIVATE
        /W4
        /wd4100
        /wd4505
        /we4715
        /we4828
        /we4473 /we4474 /we4475 /we4476 /we4477 /we4478)
endif()
