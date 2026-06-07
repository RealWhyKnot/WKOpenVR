cmake_minimum_required(VERSION 3.15)

set(CAPTIONS_PAIRDRIVER_ROOT "${CMAKE_SOURCE_DIR}")

add_library(openvr_pair_feature_captions_overlay STATIC
    HostStatusPoller.cpp
    HostStatusPoller.h
    CaptionPreviewHistory.h
    CaptionsIpcClient.cpp
    CaptionsIpcClient.h
    CaptionsConfig.cpp
    CaptionsConfig.h
    CaptionsPlugin.cpp
    CaptionsPlugin.h
    CaptionsTab.cpp
    CaptionsTab.h)

target_include_directories(openvr_pair_feature_captions_overlay PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}
    ${CAPTIONS_PAIRDRIVER_ROOT}/core/src/common
    ${CAPTIONS_PAIRDRIVER_ROOT}/core/src/overlay
    ${CMAKE_SOURCE_DIR}/lib
    ${CMAKE_SOURCE_DIR}/lib/imgui
    ${CMAKE_SOURCE_DIR}/lib/implot
    ${CMAKE_SOURCE_DIR}/lib/openvr/headers)

target_link_libraries(openvr_pair_feature_captions_overlay PRIVATE
    openvr_pair_common)

target_compile_definitions(openvr_pair_feature_captions_overlay PRIVATE
    NOMINMAX
    UNICODE
    _UNICODE)

if(MSVC)
    target_compile_options(openvr_pair_feature_captions_overlay PRIVATE
        /W4
        /wd4100
        /wd4505
        /we4715
        /we4828)
endif()
