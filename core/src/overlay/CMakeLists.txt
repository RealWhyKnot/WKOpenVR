cmake_minimum_required(VERSION 3.15)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(OPENVR_PAIRDRIVER_ROOT ${CMAKE_SOURCE_DIR} CACHE PATH "WKOpenVR checkout" FORCE)
set(OPENVR_PAIR_BUILD_UMBRELLA ON CACHE BOOL "Build feature overlay plugins for OpenVR-Pair" FORCE)

# Feature modules are gated through cmake/WKOpenVRModules.cmake: each helper
# call adds the module's overlay subdir, registers its lib for linking, and
# emits its OPENVR_PAIR_HAS_*_OVERLAY define -- unless the module ships a
# modules/<slug>/disabled-in-release.flag marker AND the build was configured
# with -DWKOPENVR_RELEASE_BUILD=ON, in which case it's skipped silently.
set(OPENVR_PAIR_OVERLAY_FEATURE_LIBS "")
set(OPENVR_PAIR_OVERLAY_DEFINES "")

wkopenvr_add_overlay_module(inputhealth)
wkopenvr_add_overlay_module(smoothing)
wkopenvr_add_overlay_module(calibration)
wkopenvr_add_overlay_module(questapp)
wkopenvr_add_overlay_module(dynamicres)
wkopenvr_add_overlay_module(facetracking)
wkopenvr_add_overlay_module(oscrouter)
wkopenvr_add_overlay_module(captions)
wkopenvr_add_overlay_module(phantom)

if(WIN32)
	enable_language(RC)
	set(OPENVR_PAIR_RC_SOURCES openvr-pair.rc)
else()
	set(OPENVR_PAIR_RC_SOURCES "")
endif()

add_library(openvr_pair_overlay_ui STATIC
	ShellSettings.cpp
	ShellSettings.h
	Theme.cpp
	Theme.h
	UiControls.cpp
	UiControls.h
	UiCore.h
	UiHelpers.h
	UiLayout.cpp
	UiLayout.h
	UiResponsive.cpp
	UiResponsive.h
	UiResponsiveLogic.h
	UiTables.cpp
	UiTables.h)

target_include_directories(openvr_pair_overlay_ui PUBLIC
	${CMAKE_CURRENT_LIST_DIR}
	${CMAKE_SOURCE_DIR}/lib
	${CMAKE_SOURCE_DIR}/lib/imgui)

target_link_libraries(openvr_pair_overlay_ui PUBLIC
	pair_imgui)

if(WIN32)
	target_link_libraries(openvr_pair_overlay_ui PUBLIC
		user32)
endif()

target_compile_definitions(openvr_pair_overlay_ui PRIVATE
	NOMINMAX
	UNICODE
	_UNICODE
	IMGUI_DEFINE_MATH_OPERATORS)

if(MSVC)
	target_compile_options(openvr_pair_overlay_ui PRIVATE /W4 /wd4100)
endif()

if(NOT WKOPENVR_RELEASE_BUILD)
	add_subdirectory(
		"${CMAKE_SOURCE_DIR}/modules/dev/src/overlay"
		"${CMAKE_BINARY_DIR}/modules/dev-overlay")
	set(OPENVR_PAIR_OVERLAY_FEATURE_LIBS
		${OPENVR_PAIR_OVERLAY_FEATURE_LIBS}
		openvr_pair_feature_dev_overlay)
	set(OPENVR_PAIR_OVERLAY_DEFINES
		${OPENVR_PAIR_OVERLAY_DEFINES}
		OPENVR_PAIR_HAS_DEV_OVERLAY=1)
endif()

add_executable(OpenVRPairOverlay WIN32
	BugReportUi.cpp
	BugReportUi.h
	CalibrationAnchor.cpp
	CalibrationAnchor.h
	DeviceFilters.cpp
	DeviceFilters.h
	FeaturePlugin.cpp
	FeaturePlugin.h
	FrameHitchLogic.h
	IpcClientBase.cpp
	IpcClientBase.h
	IpcSendQueue.cpp
	IpcSendQueue.h
	ManifestRegistration.cpp
	ManifestRegistration.h
	Migration.cpp
	Migration.h
	ModuleToggleUi.cpp
	ModuleToggleUi.h
	OverlayHotReload.cpp
	OverlayHotReload.h
	PerfStatsHub.cpp
	PerfStatsHub.h
	PerfHistory.h
	SafeModeRecovery.cpp
	SafeModeRecovery.h
	ShellContext.cpp
	ShellContext.h
	ShellFooter.cpp
	ShellFooter.h
	ShellUi.cpp
	ShellUi.h
	Theme.h
	UiHelpers.h
	UpdateNotice.cpp
	UpdateNotice.h
	UpdateNoticeLogic.h
	VrOverlayHost.cpp
	VrOverlayHost.h
	main.cpp
	${OPENVR_PAIR_RC_SOURCES})

set_target_properties(OpenVRPairOverlay PROPERTIES OUTPUT_NAME "WKOpenVR")

target_include_directories(OpenVRPairOverlay PRIVATE
	${CMAKE_CURRENT_LIST_DIR}
	${CMAKE_SOURCE_DIR}/core/src/common
	${CMAKE_SOURCE_DIR}/lib
	${CMAKE_SOURCE_DIR}/lib/openvr/headers
	${CMAKE_SOURCE_DIR}/lib/imgui
	${CMAKE_SOURCE_DIR}/lib/imgui/backends
	${CMAKE_SOURCE_DIR}/lib/implot
	${CMAKE_SOURCE_DIR}/lib/gl3w/include)

target_link_libraries(OpenVRPairOverlay PRIVATE
	openvr_pair_common
	openvr_pair_overlay_ui
	pair_imgui
	pair_implot
	pair_gl3w
	glfw
	opengl32
	shell32
	ole32
	bcrypt
	advapi32
	${OPENVR_PAIR_OVERLAY_FEATURE_LIBS})

# Dev-channel test harness (--test-harness). Linked into WKOpenVR.exe only when
# the build is not flagged as a release build, so release binaries cannot
# dispatch to or even reference the harness symbols.
if(NOT WKOPENVR_RELEASE_BUILD)
	add_subdirectory(testharness)
	target_link_libraries(OpenVRPairOverlay PRIVATE wkopenvr_testharness)
endif()

target_compile_definitions(OpenVRPairOverlay PRIVATE
	NOMINMAX
	UNICODE
	_UNICODE
	${OPENVR_PAIR_OVERLAY_DEFINES}
	"OPENVR_PAIR_VERSION_STRING=\"${WKOPENVR_VERSION_STAMP}\"")

if(MSVC)
	target_link_options(OpenVRPairOverlay PRIVATE /ENTRY:mainCRTStartup)
endif()

# Stage manifest.vrmanifest next to the umbrella exe so SteamVR can find it
# via the absolute path the exe passes to AddApplicationManifest.
add_custom_command(TARGET OpenVRPairOverlay POST_BUILD
	COMMAND ${CMAKE_COMMAND} -E copy_if_different
		"${CMAKE_CURRENT_LIST_DIR}/manifest.vrmanifest"
		"$<TARGET_FILE_DIR:OpenVRPairOverlay>/manifest.vrmanifest")

# Stage the dashboard-overlay thumbnail next to the exe. VrOverlayHost
# loads it at runtime via SetOverlayFromFile using an exe-relative path.
add_custom_command(TARGET OpenVRPairOverlay POST_BUILD
	COMMAND ${CMAKE_COMMAND} -E copy_if_different
		"${CMAKE_CURRENT_LIST_DIR}/dashboard_icon.png"
		"$<TARGET_FILE_DIR:OpenVRPairOverlay>/dashboard_icon.png")

# Stage the boundary marker glyph next to the exe. The boundary preview loads
# it at runtime for the in-VR floor/start markers via an exe-relative path.
add_custom_command(TARGET OpenVRPairOverlay POST_BUILD
	COMMAND ${CMAKE_COMMAND} -E copy_if_different
		"${CMAKE_CURRENT_LIST_DIR}/boundary_icon.png"
		"$<TARGET_FILE_DIR:OpenVRPairOverlay>/boundary_icon.png")

# Stage the face-module sync script into resources/ next to the exe.
# ModuleSyncRunner looks for it at <exe_dir>/resources/face-module-sync.ps1.
# Source ownership lives with the facetracking module; the deployed resource
# path stays shell-owned because ModuleSyncRunner resolves it next to WKOpenVR.exe.
add_custom_command(TARGET OpenVRPairOverlay POST_BUILD
	COMMAND ${CMAKE_COMMAND} -E make_directory
		"$<TARGET_FILE_DIR:OpenVRPairOverlay>/resources"
	COMMAND ${CMAKE_COMMAND} -E copy_if_different
		"${CMAKE_SOURCE_DIR}/modules/facetracking/src/overlay/face-module-sync.ps1"
		"$<TARGET_FILE_DIR:OpenVRPairOverlay>/resources/face-module-sync.ps1")

# Stage Quest App resources next to the exe. The Quest App module downloads
# platform-tools into LocalLow at runtime; only its setup script is shipped.
add_custom_command(TARGET OpenVRPairOverlay POST_BUILD
	COMMAND ${CMAKE_COMMAND} -E make_directory
		"$<TARGET_FILE_DIR:OpenVRPairOverlay>/resources/questapp"
	COMMAND ${CMAKE_COMMAND} -E copy_if_different
		"${CMAKE_SOURCE_DIR}/modules/questapp/resources/install-platform-tools.ps1"
		"$<TARGET_FILE_DIR:OpenVRPairOverlay>/resources/questapp/install-platform-tools.ps1"
	COMMAND ${CMAKE_COMMAND} -E copy_if_different
		"${CMAKE_SOURCE_DIR}/modules/questapp/resources/uninstall-questapp.ps1"
		"$<TARGET_FILE_DIR:OpenVRPairOverlay>/resources/questapp/uninstall-questapp.ps1"
	COMMAND ${CMAKE_COMMAND} -E copy_if_different
		"${CMAKE_SOURCE_DIR}/modules/questapp/resources/WKOpenVRQuestCompanion.apk"
		"$<TARGET_FILE_DIR:OpenVRPairOverlay>/resources/questapp/WKOpenVRQuestCompanion.apk")
