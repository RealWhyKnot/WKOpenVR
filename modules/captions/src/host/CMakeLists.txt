cmake_minimum_required(VERSION 3.15)

# Captions host sidecar (WKOpenVR.CaptionsHost.exe).
# Depends on: whisper.cpp (FetchContent), optional ONNX Runtime import headers/libs,
# optional CTranslate2 (vendored or prebuilt), OpenVR, WinHTTP, WASAPI.
#
# All three inference libraries are gated by CMake options so a contributor
# without the prebuilt DLLs can still build the driver+overlay with stubs.

# Belt-and-suspenders guard for the disabled-in-release path (see the
# matching note in modules/facetracking/src/host/CMakeLists.txt).
if(COMMAND wkopenvr_module_disabled)
    wkopenvr_module_disabled(captions _wk_cap_disabled)
    if(_wk_cap_disabled)
        message(STATUS "Skipping captions host module (disabled-in-release; secondary guard)")
        return()
    endif()
endif()

option(OPENVR_PAIR_BUILD_CAPTIONS_HOST
    "Build the native captions host sidecar. Off lets a C++-only contributor build without the inference library dependencies."
    ON)

if(NOT OPENVR_PAIR_BUILD_CAPTIONS_HOST)
    message(STATUS "OPENVR_PAIR_BUILD_CAPTIONS_HOST=OFF: skipping captions host build.")
    return()
endif()

# ---------------------------------------------------------------------------
# whisper.cpp (FetchContent -- MIT)
# ---------------------------------------------------------------------------
include(FetchContent)

# WHISPER_CUDA: off by default; set -DWKOPENVR_CAPTIONS_CUDA=ON to enable.
option(WKOPENVR_CAPTIONS_CUDA "Build whisper.cpp with CUDA backend" OFF)

# WHISPER_VULKAN: ON by default. Vendor-neutral GPU acceleration (NVIDIA/AMD/
# Intel) via the system Vulkan loader; only the build host needs the Vulkan SDK
# (headers + glslc shader compiler). The runtime needs the system vulkan-1.dll,
# present with any GPU driver. Set -DWKOPENVR_CAPTIONS_VULKAN=OFF for a CPU-only
# captions host when the SDK is unavailable (not recommended).
option(WKOPENVR_CAPTIONS_VULKAN "Build whisper.cpp with Vulkan backend (vendor-neutral GPU)" ON)

set(WHISPER_BUILD_TESTS    OFF CACHE BOOL "" FORCE)
set(WHISPER_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
set(GGML_CUDA ${WKOPENVR_CAPTIONS_CUDA} CACHE BOOL "" FORCE)
set(GGML_VULKAN ${WKOPENVR_CAPTIONS_VULKAN} CACHE BOOL "" FORCE)

# Fail early with an actionable message if the SDK is missing, rather than
# letting ggml-vulkan's own find_package(... REQUIRED) error deep in the tree.
if(WKOPENVR_CAPTIONS_VULKAN)
    find_package(Vulkan COMPONENTS glslc QUIET)
    if(NOT Vulkan_FOUND OR NOT Vulkan_glslc_FOUND)
        message(FATAL_ERROR
            "The captions host builds the Vulkan GPU backend by default but the Vulkan SDK "
            "(headers + glslc) was not found. Install it from https://vulkan.lunarg.com so "
            "VULKAN_SDK is set and glslc is on PATH (build.ps1 can do this for you), or build a "
            "CPU-only host with -DWKOPENVR_CAPTIONS_VULKAN=OFF (not recommended).")
    endif()
endif()

# Enable whisper.cpp/ggml ccache integration when a compiler cache was found.
# WKOPENVR_COMPILER_LAUNCHER is set by the root CMakeLists; leave it empty when
# neither sccache nor ccache is installed so the build is unaffected.
if(WKOPENVR_COMPILER_LAUNCHER)
    set(WHISPER_CCACHE ON  CACHE BOOL "" FORCE)
    set(GGML_CCACHE    ON  CACHE BOOL "" FORCE)
else()
    set(WHISPER_CCACHE OFF CACHE BOOL "" FORCE)
    set(GGML_CCACHE    OFF CACHE BOOL "" FORCE)
endif()

FetchContent_Declare(
    whisper_cpp
    GIT_REPOSITORY https://github.com/ggerganov/whisper.cpp.git
    GIT_TAG        v1.7.4
    GIT_SHALLOW    TRUE)
FetchContent_MakeAvailable(whisper_cpp)

# ---------------------------------------------------------------------------
# ONNX Runtime (prebuilt DLL -- Microsoft, MIT)
# Build uses headers/import library from lib/onnxruntime when present, or from
# a downloaded build-cache copy. The runtime DLL itself is delay-loaded and is
# installed later by the captions speech pack, so the default installer stays
# small and the host still starts without the pack.
# ---------------------------------------------------------------------------
set(ORT_ROOT "${CMAKE_SOURCE_DIR}/lib/onnxruntime")
option(WKOPENVR_CAPTIONS_FETCH_ORT
    "Fetch ONNX Runtime headers/import library into the build tree for Silero VAD support."
    ON)

if(NOT EXISTS "${ORT_ROOT}/include/onnxruntime_c_api.h" AND WKOPENVR_CAPTIONS_FETCH_ORT)
    FetchContent_Declare(
        onnxruntime_win_x64
        URL "https://github.com/microsoft/onnxruntime/releases/download/v1.26.0/onnxruntime-win-x64-1.26.0.zip"
        URL_HASH SHA256=6ebe99b5564bf4d029b6e93eac9ff423682b6212eade769e9ca3f685eaf500b4)
    FetchContent_GetProperties(onnxruntime_win_x64)
    if(NOT onnxruntime_win_x64_POPULATED)
        FetchContent_Populate(onnxruntime_win_x64)
    endif()
    set(ORT_ROOT "${onnxruntime_win_x64_SOURCE_DIR}")
endif()

if(EXISTS "${ORT_ROOT}/include/onnxruntime_c_api.h")
    set(HAVE_ORT ON)
    message(STATUS "ONNX Runtime found at ${ORT_ROOT}")
else()
    set(HAVE_ORT OFF)
    message(STATUS "ONNX Runtime not found at ${ORT_ROOT}; Silero VAD will be stubbed.")
endif()

# ---------------------------------------------------------------------------
# CTranslate2 (OpenNMT, MIT)
# Build uses headers/import library from lib/ctranslate2 when present, or a
# generated build-cache import library from the official Windows wheel. The
# runtime DLL itself is installed later by the selected translation pack.
# ---------------------------------------------------------------------------
set(CT2_ROOT "${CMAKE_SOURCE_DIR}/lib/ctranslate2")
option(WKOPENVR_CAPTIONS_FETCH_CT2
    "Fetch CTranslate2 headers/import library into the build tree for translation support."
    ON)

if(NOT EXISTS "${CT2_ROOT}/include/ctranslate2/translator.h" AND
   WKOPENVR_CAPTIONS_FETCH_CT2 AND MSVC)
    FetchContent_Declare(
        ctranslate2_source
        GIT_REPOSITORY https://github.com/OpenNMT/CTranslate2.git
        GIT_TAG        v4.7.1
        GIT_SHALLOW    TRUE)
    FetchContent_GetProperties(ctranslate2_source)
    if(NOT ctranslate2_source_POPULATED)
        FetchContent_Populate(ctranslate2_source)
    endif()

    set(CT2_WHEEL_URL
        "https://files.pythonhosted.org/packages/4a/f4/d23dbfb9c62cb642c114a30f05d753ba61d6ffbfd8a3a4012fe85a073bcb/ctranslate2-4.7.1-cp312-cp312-win_amd64.whl")
    set(CT2_WHEEL_SHA256
        "d0f734dc3757118094663bdaaf713f5090c55c1927fb330a76bb8b84173940e8")
    set(CT2_WHEEL_ROOT "${CMAKE_BINARY_DIR}/_deps/ctranslate2_wheel")
    set(CT2_WHEEL_PATH "${CT2_WHEEL_ROOT}/ctranslate2-4.7.1-cp312-cp312-win_amd64.whl")
    set(CT2_WHEEL_DLL "${CT2_WHEEL_ROOT}/ctranslate2/ctranslate2.dll")
    file(MAKE_DIRECTORY "${CT2_WHEEL_ROOT}")
    if(NOT EXISTS "${CT2_WHEEL_DLL}")
        if(NOT EXISTS "${CT2_WHEEL_PATH}")
            file(DOWNLOAD "${CT2_WHEEL_URL}" "${CT2_WHEEL_PATH}"
                EXPECTED_HASH "SHA256=${CT2_WHEEL_SHA256}"
                STATUS CT2_DOWNLOAD_STATUS
                SHOW_PROGRESS)
            list(GET CT2_DOWNLOAD_STATUS 0 CT2_DOWNLOAD_CODE)
            list(GET CT2_DOWNLOAD_STATUS 1 CT2_DOWNLOAD_MESSAGE)
            if(NOT CT2_DOWNLOAD_CODE EQUAL 0)
                message(WARNING "CTranslate2 wheel download failed: ${CT2_DOWNLOAD_MESSAGE}")
            endif()
        endif()
        if(EXISTS "${CT2_WHEEL_PATH}")
            execute_process(
                COMMAND ${CMAKE_COMMAND} -E tar xf "${CT2_WHEEL_PATH}"
                WORKING_DIRECTORY "${CT2_WHEEL_ROOT}"
                RESULT_VARIABLE CT2_EXTRACT_RESULT)
            if(NOT CT2_EXTRACT_RESULT EQUAL 0)
                message(WARNING "CTranslate2 wheel extraction failed: ${CT2_EXTRACT_RESULT}")
            endif()
        endif()
    endif()

    set(CT2_DEV_ROOT "${CMAKE_BINARY_DIR}/captions-ct2-dev")
    set(CT2_GEN_LIB "${CT2_DEV_ROOT}/lib/ctranslate2.lib")
    if(EXISTS "${ctranslate2_source_SOURCE_DIR}/include/ctranslate2/translator.h" AND
       EXISTS "${CT2_WHEEL_DLL}")
        file(MAKE_DIRECTORY
            "${CT2_DEV_ROOT}/include"
            "${CT2_DEV_ROOT}/bin"
            "${CT2_DEV_ROOT}/lib")
        file(COPY "${ctranslate2_source_SOURCE_DIR}/include/"
            DESTINATION "${CT2_DEV_ROOT}/include")
        file(COPY "${CT2_WHEEL_DLL}"
            DESTINATION "${CT2_DEV_ROOT}/bin")

        if(NOT EXISTS "${CT2_GEN_LIB}")
            set(CT2_DEF "${CT2_DEV_ROOT}/lib/ctranslate2.def")
            execute_process(
                COMMAND "${CMAKE_LINKER}" /dump /exports "${CT2_WHEEL_DLL}"
                OUTPUT_VARIABLE CT2_EXPORTS
                RESULT_VARIABLE CT2_DUMP_RESULT)
            if(CT2_DUMP_RESULT EQUAL 0)
                string(REGEX MATCHALL
                    "\n[ \t]*[0-9]+[ \t]+[0-9A-Fa-f]+[ \t]+[0-9A-Fa-f]+[ \t]+[^ \t\r\n]+"
                    CT2_EXPORT_LINES "${CT2_EXPORTS}")
                file(WRITE "${CT2_DEF}" "LIBRARY ctranslate2.dll\nEXPORTS\n")
                foreach(CT2_EXPORT_LINE IN LISTS CT2_EXPORT_LINES)
                    string(REGEX REPLACE
                        "^[ \t\r\n]*[0-9]+[ \t]+[0-9A-Fa-f]+[ \t]+[0-9A-Fa-f]+[ \t]+([^ \t\r\n]+).*$"
                        "\\1"
                        CT2_SYMBOL
                        "${CT2_EXPORT_LINE}")
                    file(APPEND "${CT2_DEF}" "  ${CT2_SYMBOL}\n")
                endforeach()
                execute_process(
                    COMMAND "${CMAKE_AR}" "/def:${CT2_DEF}" /machine:x64 "/out:${CT2_GEN_LIB}"
                    RESULT_VARIABLE CT2_LIB_RESULT)
                if(NOT CT2_LIB_RESULT EQUAL 0)
                    message(WARNING "CTranslate2 import library generation failed: ${CT2_LIB_RESULT}")
                endif()
            else()
                message(WARNING "Could not read exports from CTranslate2 DLL: ${CT2_DUMP_RESULT}")
            endif()
        endif()

        if(EXISTS "${CT2_GEN_LIB}")
            set(CT2_ROOT "${CT2_DEV_ROOT}")
        endif()
    endif()
endif()

if(EXISTS "${CT2_ROOT}/include/ctranslate2/translator.h")
    set(HAVE_CT2 ON)
    message(STATUS "CTranslate2 found at ${CT2_ROOT}")
else()
    set(HAVE_CT2 OFF)
    message(STATUS "CTranslate2 not found at ${CT2_ROOT}; translation will be stubbed.")
endif()

# ---------------------------------------------------------------------------
# SentencePiece (Google, Apache-2.0)
# OPUS-MT CTranslate2 models expect SentencePiece tokenized input and output.
# CTranslate2 intentionally accepts tokens, not raw text, so the host needs the
# tokenizer whenever translation is compiled in.
# ---------------------------------------------------------------------------
option(WKOPENVR_CAPTIONS_FETCH_SENTENCEPIECE
    "Fetch SentencePiece for OPUS-MT tokenization when translation support is enabled."
    ON)

set(HAVE_SENTENCEPIECE OFF)
if(HAVE_CT2 AND WKOPENVR_CAPTIONS_FETCH_SENTENCEPIECE)
    set(SPM_BUILD_TEST OFF CACHE BOOL "" FORCE)
    set(SPM_ENABLE_SHARED OFF CACHE BOOL "" FORCE)
    set(SPM_ENABLE_TCMALLOC OFF CACHE BOOL "" FORCE)
    FetchContent_Declare(
        sentencepiece
        GIT_REPOSITORY https://github.com/google/sentencepiece.git
        GIT_TAG        v0.2.1
        GIT_SHALLOW    TRUE)
    FetchContent_MakeAvailable(sentencepiece)
    if(TARGET sentencepiece)
        set(HAVE_SENTENCEPIECE ON)
        set(SENTENCEPIECE_INCLUDE_DIRS
            "${sentencepiece_SOURCE_DIR}/src"
            "${sentencepiece_BINARY_DIR}/src")
        message(STATUS "SentencePiece found via FetchContent")
    else()
        message(WARNING "SentencePiece target was not created; translation will be disabled.")
    endif()
endif()

# ---------------------------------------------------------------------------
# Host executable
# ---------------------------------------------------------------------------
set(CAPTIONS_PAIRDRIVER_ROOT "${CMAKE_SOURCE_DIR}")
set(OPENVR_LIB_DIR "${CMAKE_SOURCE_DIR}/lib/openvr")

find_library(OPENVR_LIBRARIES
    NAMES openvr_api
    PATHS ${OPENVR_LIB_DIR}/bin ${OPENVR_LIB_DIR}/lib
    PATH_SUFFIXES win64
    NO_DEFAULT_PATH NO_CMAKE_FIND_ROOT_PATH)

set(HOST_SOURCES
    ActionBindings.cpp
    ActionBindings.h
    ChatboxPacer.cpp
    ChatboxPacer.h
    HostStatus.cpp
    HostStatus.h
    Logging.cpp
    Logging.h
    ModelDownloader.cpp
    ModelDownloader.h
    RouterPublisher.cpp
    RouterPublisher.h
    SileroVad.cpp
    SileroVad.h
    SileroVadModelContract.h
    Captions.cpp
    Captions.h
    ChatboxText.h
    WasapiCapture.cpp
    WasapiCapture.h
    TranscriptText.h
    WhisperEngine.cpp
    WhisperEngine.h
    main.cpp)

add_executable(WKOpenVRCaptionsHost ${HOST_SOURCES})
set_target_properties(WKOpenVRCaptionsHost PROPERTIES
    OUTPUT_NAME "WKOpenVR.CaptionsHost"
    WIN32_EXECUTABLE TRUE)

target_compile_definitions(WKOpenVRCaptionsHost PRIVATE
    NOMINMAX
    UNICODE)
if(WKOPENVR_CAPTIONS_CUDA)
    target_compile_definitions(WKOpenVRCaptionsHost PRIVATE
        WKOPENVR_CAPTIONS_CUDA_ENABLED=1)
endif()
if(WKOPENVR_CAPTIONS_VULKAN)
    target_compile_definitions(WKOpenVRCaptionsHost PRIVATE
        WKOPENVR_CAPTIONS_VULKAN_ENABLED=1)
endif()

target_compile_features(WKOpenVRCaptionsHost PRIVATE cxx_std_17)

# WIN32_EXECUTABLE sets SUBSYSTEM:WINDOWS (no console window) but expects WinMain.
# Override the entry point so int main() is used as-is.
if(MSVC)
    target_link_options(WKOpenVRCaptionsHost PRIVATE /ENTRY:mainCRTStartup)

    # Delay-load optional inference DLLs so the host can still report setup
    # status when a runtime pack has not been installed yet.
    set(CAPTIONS_DELAYLOAD_DLLS
        /DELAYLOAD:onnxruntime.dll
        /DELAYLOAD:ctranslate2.dll)
    if(WKOPENVR_CAPTIONS_CUDA)
        list(APPEND CAPTIONS_DELAYLOAD_DLLS
            /DELAYLOAD:cublas64_13.dll
            /DELAYLOAD:cublasLt64_13.dll
            /DELAYLOAD:cudart64_13.dll
            /DELAYLOAD:nvcudart_hybrid64.dll
            /DELAYLOAD:nvcuda.dll)
    endif()
    target_link_options(WKOpenVRCaptionsHost PRIVATE
        /NODEFAULTLIB:LIBCMT
        /IGNORE:4199
        ${CAPTIONS_DELAYLOAD_DLLS})
    target_link_libraries(WKOpenVRCaptionsHost PRIVATE delayimp)
endif()

target_include_directories(WKOpenVRCaptionsHost PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}
    ${CAPTIONS_PAIRDRIVER_ROOT}/core/src/common
    ${OPENVR_LIB_DIR}/headers
    ${CAPTIONS_PAIRDRIVER_ROOT}/lib)

target_link_libraries(WKOpenVRCaptionsHost PRIVATE
    openvr_pair_common
    whisper
    ${OPENVR_LIBRARIES}
    ole32
    shell32
    winhttp
    mmdevapi)

# Link the Vulkan loader for the host's own device pre-probe (the guarded
# vkEnumeratePhysicalDevices in main.cpp). ggml links it transitively too, but
# the host calls Vulkan directly to decide whether GPU is safe to request.
if(WKOPENVR_CAPTIONS_VULKAN)
    target_include_directories(WKOpenVRCaptionsHost PRIVATE ${Vulkan_INCLUDE_DIRS})
    target_link_libraries(WKOpenVRCaptionsHost PRIVATE Vulkan::Vulkan)
endif()

# ONNX Runtime.
if(HAVE_ORT)
    target_include_directories(WKOpenVRCaptionsHost PRIVATE ${ORT_ROOT}/include)
    find_library(ORT_LIB onnxruntime PATHS "${ORT_ROOT}/lib" NO_DEFAULT_PATH)
    if(ORT_LIB)
        target_link_libraries(WKOpenVRCaptionsHost PRIVATE ${ORT_LIB})
        target_compile_definitions(WKOpenVRCaptionsHost PRIVATE HAVE_ORT=1)
    endif()
endif()

# CTranslate2.
if(HAVE_CT2)
    target_include_directories(WKOpenVRCaptionsHost PRIVATE ${CT2_ROOT}/include)
    find_library(CT2_LIB ctranslate2 PATHS "${CT2_ROOT}/lib" NO_DEFAULT_PATH)
    if(CT2_LIB)
        target_link_libraries(WKOpenVRCaptionsHost PRIVATE ${CT2_LIB})
        target_compile_definitions(WKOpenVRCaptionsHost PRIVATE HAVE_CT2=1)
    endif()
endif()

if(HAVE_SENTENCEPIECE)
    target_include_directories(WKOpenVRCaptionsHost PRIVATE ${SENTENCEPIECE_INCLUDE_DIRS})
    target_link_libraries(WKOpenVRCaptionsHost PRIVATE sentencepiece)
    target_compile_definitions(WKOpenVRCaptionsHost PRIVATE HAVE_SENTENCEPIECE=1)
endif()

# ---------------------------------------------------------------------------
# Stage resources (silero_vad.onnx) and bindings into the build tree.
# ---------------------------------------------------------------------------
set(CAPTIONS_HOST_STAGE_DIR
    "${CMAKE_BINARY_DIR}/driver_wkopenvr/resources/captions/host")

add_custom_command(TARGET WKOpenVRCaptionsHost POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E make_directory "${CAPTIONS_HOST_STAGE_DIR}"
    COMMAND ${CMAKE_COMMAND} -E make_directory "${CAPTIONS_HOST_STAGE_DIR}/resources"
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        "$<TARGET_FILE:WKOpenVRCaptionsHost>"
        "${CAPTIONS_HOST_STAGE_DIR}/WKOpenVR.CaptionsHost.exe"
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        "${OPENVR_LIB_DIR}/bin/win64/openvr_api.dll"
        "${CAPTIONS_HOST_STAGE_DIR}/openvr_api.dll"
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        "${CMAKE_CURRENT_LIST_DIR}/actions.json"
        "${CAPTIONS_HOST_STAGE_DIR}/actions.json"
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        "${CMAKE_CURRENT_LIST_DIR}/captions.vrmanifest"
        "${CAPTIONS_HOST_STAGE_DIR}/captions.vrmanifest"
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        "${CMAKE_CURRENT_LIST_DIR}/bindings_knuckles.json"
        "${CAPTIONS_HOST_STAGE_DIR}/bindings_knuckles.json"
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        "${CMAKE_CURRENT_LIST_DIR}/bindings_touch.json"
        "${CAPTIONS_HOST_STAGE_DIR}/bindings_touch.json"
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        "${CMAKE_CURRENT_LIST_DIR}/bindings_vive.json"
        "${CAPTIONS_HOST_STAGE_DIR}/bindings_vive.json"
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        "${CMAKE_CURRENT_LIST_DIR}/resources/captions-packs.json"
        "${CAPTIONS_HOST_STAGE_DIR}/resources/captions-packs.json"
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        "${CMAKE_CURRENT_LIST_DIR}/resources/install-captions-pack.ps1"
        "${CAPTIONS_HOST_STAGE_DIR}/resources/install-captions-pack.ps1"
    COMMENT "Staging WKOpenVR.CaptionsHost")

# Copy silero_vad.onnx if it exists in the source tree.
if(EXISTS "${CMAKE_CURRENT_LIST_DIR}/resources/silero_vad.onnx")
    add_custom_command(TARGET WKOpenVRCaptionsHost POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
            "${CMAKE_CURRENT_LIST_DIR}/resources/silero_vad.onnx"
            "${CAPTIONS_HOST_STAGE_DIR}/resources/silero_vad.onnx")
endif()

adjust_bin_paths(WKOpenVRCaptionsHost)

set_target_properties(WKOpenVRCaptionsHost PROPERTIES
    FOLDER "modules/captions")
