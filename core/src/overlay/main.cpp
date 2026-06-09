#include "BuildChannel.h"
#include "DashboardInputRuntimeGate.h"
#include "DebugLogging.h"
#include "DiagnosticsLog.h"
#include "FeaturePlugin.h"
#include "ManifestRegistration.h"
#include "Migration.h"
#include "ModuleRegistry.h"
#include "ProcessPerfLog.h"
#include "RuntimeHealthSummary.h"
#include "SafeModeRecovery.h"
#include "ShellContext.h"
#include "ShellUi.h"
#include "Theme.h"
#include "UiHelpers.h"
#include "UpdateNotice.h"
#include "VrOverlayHost.h"

#if WKOPENVR_BUILD_IS_DEV
#include "testharness/TestHarnessRunner.h"
#endif

#include <GL/gl3w.h>
#include <GLFW/glfw3.h>
#ifdef _WIN32
#define GLFW_EXPOSE_NATIVE_WIN32
#include <GLFW/glfw3native.h>
#endif
#include <imgui.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>
#include <implot.h>

#ifdef _WIN32
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <windows.h>
#endif

#include <algorithm>
#include <cstdio>
#include <array>
#include <exception>
#include <memory>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#ifndef OPENVR_PAIR_VERSION_STRING
#define OPENVR_PAIR_VERSION_STRING "0.0.0.0-dev"
#endif

namespace openvr_pair::overlay {

std::unique_ptr<FeaturePlugin> CreateInputHealthPlugin();
std::unique_ptr<FeaturePlugin> CreateSmoothingPlugin();
std::unique_ptr<FeaturePlugin> CreateDashboardInputPlugin();
std::unique_ptr<FeaturePlugin> CreateSpaceCalibratorPlugin();
std::unique_ptr<FeaturePlugin> CreateQuestAppPlugin();
std::unique_ptr<FeaturePlugin> CreateFaceTrackingPlugin();
#if OPENVR_PAIR_HAS_OSCROUTER_OVERLAY
std::unique_ptr<FeaturePlugin> CreateOscRouterPlugin();
#endif
#if OPENVR_PAIR_HAS_CAPTIONS_OVERLAY
std::unique_ptr<FeaturePlugin> CreateCaptionsPlugin();
#endif
#if OPENVR_PAIR_HAS_PHANTOM_OVERLAY
std::unique_ptr<FeaturePlugin> CreatePhantomPlugin();
#endif
#if OPENVR_PAIR_HAS_DEV_OVERLAY
std::unique_ptr<FeaturePlugin> CreateDevPlugin(std::vector<FeaturePlugin*> plugins);
#endif

} // namespace openvr_pair::overlay

namespace {

void GlfwErrorCallback(int code, const char* description)
{
	fprintf(stderr, "[glfw] error %d: %s\n", code, description ? description : "(null)");
	openvr_pair::common::DiagnosticLog("overlay", "glfw_error code=%d description='%s'", code,
	                                   description ? description : "(null)");
}

std::vector<std::unique_ptr<openvr_pair::overlay::FeaturePlugin>> CreatePlugins()
{
	using namespace openvr_pair::overlay;
	std::vector<std::unique_ptr<FeaturePlugin>> plugins;
#if OPENVR_PAIR_HAS_INPUTHEALTH_OVERLAY
	plugins.push_back(CreateInputHealthPlugin());
#endif
#if OPENVR_PAIR_HAS_SMOOTHING_OVERLAY
	plugins.push_back(CreateSmoothingPlugin());
#endif
#if OPENVR_PAIR_HAS_DASHBOARDINPUT_OVERLAY
	plugins.push_back(CreateDashboardInputPlugin());
#endif
#if OPENVR_PAIR_HAS_CALIBRATION_OVERLAY
	plugins.push_back(CreateSpaceCalibratorPlugin());
#endif
#if OPENVR_PAIR_HAS_QUESTAPP_OVERLAY
	plugins.push_back(CreateQuestAppPlugin());
#endif
#if OPENVR_PAIR_HAS_FACETRACKING_OVERLAY
	plugins.push_back(CreateFaceTrackingPlugin());
#endif
#if OPENVR_PAIR_HAS_OSCROUTER_OVERLAY
	plugins.push_back(CreateOscRouterPlugin());
#endif
#if OPENVR_PAIR_HAS_CAPTIONS_OVERLAY
	plugins.push_back(CreateCaptionsPlugin());
#endif
#if OPENVR_PAIR_HAS_PHANTOM_OVERLAY
	plugins.push_back(CreatePhantomPlugin());
#endif
#if OPENVR_PAIR_HAS_DEV_OVERLAY
	std::vector<FeaturePlugin*> devPluginSources;
	devPluginSources.reserve(plugins.size());
	for (auto& plugin : plugins) {
		devPluginSources.push_back(plugin.get());
	}
	plugins.push_back(CreateDevPlugin(std::move(devPluginSources)));
#endif
	return plugins;
}

class CompositorTimingSampler
{
public:
	void MaybeSample(double nowSeconds)
	{
		if (lastSampleSeconds_ > 0.0 && nowSeconds >= lastSampleSeconds_ && nowSeconds - lastSampleSeconds_ < 1.0) {
			return;
		}
		lastSampleSeconds_ = nowSeconds;

		vr::IVRCompositor* compositor = vr::VRCompositor();
		if (!compositor) return;

		std::array<vr::Compositor_FrameTiming, 128> timings{};
		for (auto& timing : timings) {
			timing.m_nSize = sizeof(vr::Compositor_FrameTiming);
		}

		const uint32_t count = compositor->GetFrameTimings(timings.data(), static_cast<uint32_t>(timings.size()));
		if (count == 0) return;

		uint32_t maxFrameIndex = lastFrameIndex_;
		for (uint32_t i = 0; i < count && i < timings.size(); ++i) {
			const vr::Compositor_FrameTiming& timing = timings[i];
			if (haveLastFrame_ && timing.m_nFrameIndex <= lastFrameIndex_) {
				continue;
			}

			openvr_pair::common::RuntimeCompositorTimingSample sample{};
			sample.frameIndex = timing.m_nFrameIndex;
			sample.framePresents = timing.m_nNumFramePresents;
			sample.droppedFrames = timing.m_nNumDroppedFrames;
			sample.mispresentedFrames = timing.m_nNumMisPresented;
			sample.reprojectionFlags = timing.m_nReprojectionFlags;
			sample.clientFrameIntervalMs = timing.m_flClientFrameIntervalMs;
			sample.totalRenderGpuMs = timing.m_flTotalRenderGpuMs;
			sample.compositorRenderGpuMs = timing.m_flCompositorRenderGpuMs;
			sample.compositorRenderCpuMs = timing.m_flCompositorRenderCpuMs;
			sample.submitFrameMs = timing.m_flSubmitFrameMs;
			sample.hmdPoseValid = timing.m_HmdPose.bPoseIsValid;
			sample.hmdTrackingResult = static_cast<int>(timing.m_HmdPose.eTrackingResult);
			openvr_pair::common::RecordRuntimeCompositorTiming(sample);

			maxFrameIndex = std::max(maxFrameIndex, timing.m_nFrameIndex);
		}

		haveLastFrame_ = true;
		lastFrameIndex_ = maxFrameIndex;
	}

private:
	double lastSampleSeconds_ = 0.0;
	bool haveLastFrame_ = false;
	uint32_t lastFrameIndex_ = 0;
};

} // namespace

int main(int argc, char** argv)
{
	using namespace openvr_pair::overlay;

	// Headless command modes. The installer calls --register-only as a
	// post-install step, and the uninstaller calls --unregister-only before
	// deleting the exe so SteamVR does not end up holding an autolaunch
	// pointer at a deleted binary. Both exit before GLFW touches the screen.
	bool registerOnly = false;
	bool unregisterOnly = false;
	bool testHarness = false;
	for (int i = 1; i < argc; ++i) {
		const std::string_view arg(argv[i]);
		if (arg == "--register-only") registerOnly = true;
		if (arg == "--unregister-only") unregisterOnly = true;
		if (arg == "--test-harness") testHarness = true;
	}

	openvr_pair::common::DiagnosticLog("overlay",
	                                   "startup version=%s build=%s channel=%s argc=%d register_only=%d "
	                                   "unregister_only=%d test_harness=%d debug_enabled=%d debug_forced=%d",
	                                   OPENVR_PAIR_VERSION_STRING, WKOPENVR_BUILD_STAMP, WKOPENVR_BUILD_CHANNEL, argc,
	                                   registerOnly ? 1 : 0, unregisterOnly ? 1 : 0, testHarness ? 1 : 0,
	                                   openvr_pair::common::IsDebugLoggingEnabled() ? 1 : 0,
	                                   openvr_pair::common::IsDebugLoggingForcedOn() ? 1 : 0);

#if WKOPENVR_BUILD_IS_DEV
	if (testHarness) {
		openvr_pair::common::DiagnosticLog("overlay", "entering test harness");
		return openvr_pair::overlay::testharness::Run(argc, argv);
	}
#else
	if (testHarness) {
		openvr_pair::common::DiagnosticLog("overlay", "test harness requested on non-dev build");
		fprintf(stderr, "--test-harness is only available on dev builds (current channel: %s)\n",
		        WKOPENVR_BUILD_CHANNEL);
		return 2;
	}
#endif

	if (unregisterOnly) {
		openvr_pair::common::DiagnosticLog("overlay", "unregister-only requested");
		UnregisterApplicationManifest();
		return 0;
	}

	// First-launch migration: copy AppData tree and SC registry key from
	// the old OpenVR-Pair paths to WKOpenVR. Idempotent -- short-circuits
	// immediately once the new locations already exist.
	RunFirstLaunchMigration();

	if (registerOnly) {
		openvr_pair::common::DiagnosticLog("overlay", "register-only requested");
		RegisterApplicationManifest(true);
		return 0;
	}

	// Register vrmanifest with SteamVR if it is already reachable. Idempotent;
	// normal desktop launches use a no-launch probe so WKOpenVR does not wake
	// SteamVR just to open the desktop window.
	RegisterApplicationManifest(false);

	// Post-crash self-heal. If SteamVR is running but safe mode has blocked the
	// WKOpenVR driver, and the prior crash is attributable to one of our own
	// modules, keep that module disabled, re-enable the blocked add-ons, and
	// relaunch SteamVR -- then exit so the fresh SteamVR auto-launches a clean
	// overlay. No-op on a normal launch.
	SafeModeRecoveryResult safeModeResult = RunSafeModeRecoveryIfNeeded();
	if (safeModeResult.relaunchedSteamVr) {
		openvr_pair::common::DiagnosticLog("overlay", "exiting after safe-mode recovery relaunch");
		return 0;
	}

	glfwSetErrorCallback(GlfwErrorCallback);
	if (!glfwInit()) {
		openvr_pair::common::DiagnosticLog("overlay", "glfw_init_failed");
		return 1;
	}

	glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
	glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
	glfwWindowHint(GLFW_RESIZABLE, GLFW_TRUE);

	GLFWwindow* window = glfwCreateWindow(1200, 780, "WKOpenVR", nullptr, nullptr);
	if (!window) {
		openvr_pair::common::DiagnosticLog("overlay", "glfw_create_window_failed size=1200x780");
		glfwTerminate();
		return 1;
	}

	// Floor the window size so the Modules table's fixed Status/Enabled
	// columns always have room and the tab strip remains usable. The
	// dashboard overlay continues to render at a fixed 1200x780 regardless
	// of the desktop window size, so this only affects monitor-side use.
	glfwSetWindowSizeLimits(window, 640, 480, GLFW_DONT_CARE, GLFW_DONT_CARE);

#ifdef _WIN32
	// Windows shell (taskbar, Start menu, alt-tab) picks the ICON resource
	// off the .exe directly, but the GLFW window's title-bar icon comes
	// from a separate per-window WM_SETICON message. Without this, the
	// title bar renders GLFW's default icon while the taskbar shows the
	// real one -- which is exactly the asymmetry the user reported.
	// LoadImageW with LR_SHARED lets Windows manage the HICON lifetime.
	{
		HWND hwnd = glfwGetWin32Window(window);
		HINSTANCE hinst = GetModuleHandleW(nullptr);
		HICON iconBig = (HICON)LoadImageW(hinst, MAKEINTRESOURCEW(1), IMAGE_ICON, 0, 0, LR_DEFAULTSIZE | LR_SHARED);
		HICON iconSmall = (HICON)LoadImageW(hinst, MAKEINTRESOURCEW(1), IMAGE_ICON, GetSystemMetrics(SM_CXSMICON),
		                                    GetSystemMetrics(SM_CYSMICON), LR_SHARED);
		if (hwnd && iconBig) SendMessageW(hwnd, WM_SETICON, ICON_BIG, (LPARAM)iconBig);
		if (hwnd && iconSmall) SendMessageW(hwnd, WM_SETICON, ICON_SMALL, (LPARAM)iconSmall);
	}
#endif

	glfwMakeContextCurrent(window);
	glfwSwapInterval(1);
	if (gl3wInit() != 0) {
		openvr_pair::common::DiagnosticLog("overlay", "gl3w_init_failed");
		glfwDestroyWindow(window);
		glfwTerminate();
		return 1;
	}

	IMGUI_CHECKVERSION();
	ImGui::CreateContext();
	ImPlot::CreateContext();
	// Theme colors are applied by InitThemeFromDisk further down; no
	// StyleColorsDark() call needed here. ApplyOverlayStyle still owns
	// padding/spacing/rounding (theme-independent).
	ImGui::GetIO().IniFilename = nullptr;

	ImGui_ImplGlfw_InitForOpenGL(window, true);
	ImGui_ImplOpenGL3_Init("#version 130");

	// Two render targets, one chosen per frame:
	//
	//  - vrFbo (fixed 1200x780): submitted to the SteamVR dashboard. The
	//    fixed size keeps the in-VR overlay's apparent resolution and the
	//    SetOverlayMouseScale mapping stable across desktop-window resizes.
	//
	//  - winFbo (matches the GLFW framebuffer, reallocated on resize):
	//    used when the dashboard is not visible so the desktop window
	//    blits 1:1, no stretching, and ImGui lays out at the actual
	//    window size.
	constexpr int kVrFboWidth = 1200;
	constexpr int kVrFboHeight = 780;

	auto allocFboTexture = [](GLuint& fbo, GLuint& tex, int w, int h) {
		glGenTextures(1, &tex);
		glBindTexture(GL_TEXTURE_2D, tex);
		glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, w, h, 0, GL_RGBA, GL_UNSIGNED_BYTE, nullptr);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
		glGenFramebuffers(1, &fbo);
		glBindFramebuffer(GL_FRAMEBUFFER, fbo);
		glFramebufferTexture(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, tex, 0);
		GLenum drawBuffers[1] = {GL_COLOR_ATTACHMENT0};
		glDrawBuffers(1, drawBuffers);
		if (glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE) {
			fprintf(stderr, "[WKOpenVR] framebuffer incomplete (%dx%d)\n", w, h);
		}
		glBindFramebuffer(GL_FRAMEBUFFER, 0);
	};

	GLuint vrFbo = 0, vrTexture = 0;
	allocFboTexture(vrFbo, vrTexture, kVrFboWidth, kVrFboHeight);

	int initialFbw = 0, initialFbh = 0;
	glfwGetFramebufferSize(window, &initialFbw, &initialFbh);
	if (initialFbw <= 0) initialFbw = kVrFboWidth;
	if (initialFbh <= 0) initialFbh = kVrFboHeight;
	GLuint winFbo = 0, winTexture = 0;
	allocFboTexture(winFbo, winTexture, initialFbw, initialFbh);
	int curWinFbW = initialFbw;
	int curWinFbH = initialFbh;

	auto reallocWinFbo = [&](int w, int h) {
		// Only the texture storage needs to change; the FBO handle and
		// the texture attachment stay valid because the binding tracks
		// the texture name, not its dimensions.
		glBindTexture(GL_TEXTURE_2D, winTexture);
		glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, w, h, 0, GL_RGBA, GL_UNSIGNED_BYTE, nullptr);
		curWinFbW = w;
		curWinFbH = h;
	};

	ShellContext context = CreateShellContext();
	if (safeModeResult.surfaceNotice && !safeModeResult.noticeMessage.empty()) {
		context.SetStatus(safeModeResult.noticeMessage, 120.0);
	}
	openvr_pair::overlay::ui::ApplyOverlayStyle();
	openvr_pair::overlay::ui::InitThemeFromDisk(context);
	openvr_pair::common::DiagnosticLog(
	    "overlay", "shell_context_ready driver_resource_dirs=%zu log_root_set=%d profile_root_set=%d",
	    context.driverResourceDirs.size(), context.logRoot.empty() ? 0 : 1, context.profileRoot.empty() ? 0 : 1);

	// Fire the GitHub-release probe once. The worker is non-blocking; the
	// ShellFooter polls GetUpdateNoticeState() and renders an indicator
	// only after the probe returns AND a newer release exists. Dev builds
	// (version string contains "-XXXX") short-circuit inside the worker
	// and never hit the network.
	openvr_pair::overlay::StartUpdateCheck();

	auto plugins = CreatePlugins();
	openvr_pair::common::DiagnosticLog("overlay", "plugins_created count=%zu", plugins.size());
	for (auto& plugin : plugins) {
		const bool installed = plugin->IsInstalled(context);
		openvr_pair::common::DiagnosticLog("overlay", "plugin_state name='%s' flag='%s' pipe='%s' installed=%d",
		                                   plugin->Name(), plugin->FlagFileName(), plugin->PipeName(),
		                                   installed ? 1 : 0);
		plugin->OnStart(context);
		openvr_pair::common::DiagnosticLog("overlay", "plugin_started name='%s' installed=%d", plugin->Name(),
		                                   installed ? 1 : 0);
	}

	auto vrOverlay = std::make_unique<VrOverlayHost>();
	bool haveVrState = false;
	bool prevActiveDashboardOverlay = false;
	bool prevAnyDashboardVisible = false;
	bool prevVrConnected = false;
	bool prevSafeOverlayVisible = false;
	bool prevSafeOverlayInputReady = false;
	bool prevSafeOverlayGlobalPriorityEnabled = false;
	std::string prevSafeOverlayStatus;
	int prevPrimaryDashboardHand = 0;
	openvr_pair::common::ProcessPerfSampler perfSampler;
	CompositorTimingSampler compositorSampler;

	while (!glfwWindowShouldClose(window) && !vrOverlay->QuitRequested()) {
		openvr_pair::common::ProcessPerfSample perfSample{};
		if (perfSampler.MaybeSample(perfSample)) {
			openvr_pair::common::RecordRuntimeProcessSample("overlay", perfSample);
			if (openvr_pair::common::IsDebugLoggingEnabled()) {
				const std::string line = openvr_pair::common::FormatProcessPerfSample("overlay", perfSample);
				openvr_pair::common::DiagnosticLog("perf", "%s", line.c_str());
			}
			openvr_pair::common::MaybeWriteRuntimeHealthSummary();
		}

		context.TickToggles();

		ImGui_ImplOpenGL3_NewFrame();
		ImGui_ImplGlfw_NewFrame();

		// Drain SteamVR overlay events AFTER ImGui_ImplGlfw_NewFrame
		// so the VR mouse position wins over GLFW's desktop cursor
		// while the dashboard is visible. ImGui processes the event
		// queue in order on NewFrame; later events override earlier
		// ones. When the dashboard is not visible no mouse events
		// fire and GLFW's position is used unchanged.
		const bool dashboardInputEnabled = context.IsFlagPresent(
		    openvr_pair::common::modules::FlagFileName(openvr_pair::common::modules::ModuleId::DashboardInput));
		const bool dashboardInputRuntimeEnabled = openvr_pair::common::dashboardinput::RuntimeEnabled(
		    dashboardInputEnabled,
		    context.IsFlagPresent(openvr_pair::common::dashboardinput::kRuntimeOptInFlagFileName));
		vrOverlay->SetSafeOverlayEnabled(dashboardInputRuntimeEnabled);
		const bool activeDashboardOverlay = vrOverlay->TickFrame(kVrFboWidth, kVrFboHeight);
		const bool anyDashboardVisible = vrOverlay->AnyDashboardVisible();
		const bool safeOverlayVisible = vrOverlay->SafeOverlayVisible();
		const bool vrSurfaceVisible = activeDashboardOverlay || safeOverlayVisible;
		context.vrConnected = vrOverlay->VrConnected();
		context.activeDashboardOverlay = activeDashboardOverlay;
		context.anyDashboardVisible = anyDashboardVisible;
		context.primaryDashboardDevice = vrOverlay->PrimaryDashboardDevice();
		context.primaryDashboardHand = vrOverlay->PrimaryDashboardHand();
		context.dashboardVisible = activeDashboardOverlay;
		context.dashboardInputSafeOverlayVisible = safeOverlayVisible;
		context.dashboardInputSafeOverlayInputReady = vrOverlay->SafeOverlayInputReady();
		context.dashboardInputSafeOverlayGlobalPriorityEnabled = vrOverlay->SafeOverlayGlobalPriorityEnabled();
		context.dashboardInputSafeOverlayStatus = vrOverlay->SafeOverlayStatus();
		if (context.vrConnected) {
			compositorSampler.MaybeSample(glfwGetTime());
		}
		if (!haveVrState || activeDashboardOverlay != prevActiveDashboardOverlay ||
		    anyDashboardVisible != prevAnyDashboardVisible || context.vrConnected != prevVrConnected ||
		    safeOverlayVisible != prevSafeOverlayVisible ||
		    context.dashboardInputSafeOverlayInputReady != prevSafeOverlayInputReady ||
		    context.dashboardInputSafeOverlayGlobalPriorityEnabled != prevSafeOverlayGlobalPriorityEnabled ||
		    context.dashboardInputSafeOverlayStatus != prevSafeOverlayStatus ||
		    context.primaryDashboardHand != prevPrimaryDashboardHand) {
			openvr_pair::common::DiagnosticLog(
			    "overlay",
			    "vr_state active_dashboard_overlay=%d any_dashboard_visible=%d vr_connected=%d "
			    "primary_dashboard_device=%u primary_dashboard_hand=%d safe_overlay_visible=%d "
			    "safe_input_ready=%d safe_global_priority=%d safe_status='%s'",
			    activeDashboardOverlay ? 1 : 0, anyDashboardVisible ? 1 : 0, context.vrConnected ? 1 : 0,
			    context.primaryDashboardDevice, context.primaryDashboardHand, safeOverlayVisible ? 1 : 0,
			    context.dashboardInputSafeOverlayInputReady ? 1 : 0,
			    context.dashboardInputSafeOverlayGlobalPriorityEnabled ? 1 : 0,
			    context.dashboardInputSafeOverlayStatus.c_str());
			haveVrState = true;
			prevActiveDashboardOverlay = activeDashboardOverlay;
			prevAnyDashboardVisible = anyDashboardVisible;
			prevVrConnected = context.vrConnected;
			prevSafeOverlayVisible = safeOverlayVisible;
			prevSafeOverlayInputReady = context.dashboardInputSafeOverlayInputReady;
			prevSafeOverlayGlobalPriorityEnabled = context.dashboardInputSafeOverlayGlobalPriorityEnabled;
			prevSafeOverlayStatus = context.dashboardInputSafeOverlayStatus;
			prevPrimaryDashboardHand = context.primaryDashboardHand;
		}

		for (auto& plugin : plugins) {
			if (plugin->IsInstalled(context)) plugin->Tick(context);
		}

		ImGuiIO& io = ImGui::GetIO();
		if (vrSurfaceVisible) {
			// VR render target is fixed-resolution; override what GLFW
			// reported so ImGui lays out at the FBO size and the VR mouse
			// coords (which are in submitted-texture pixel space) map back
			// onto ImGui widgets correctly.
			io.DisplaySize = ImVec2(static_cast<float>(kVrFboWidth), static_cast<float>(kVrFboHeight));
			io.DisplayFramebufferScale = ImVec2(1.0f, 1.0f);
			io.ConfigFlags |= ImGuiConfigFlags_NoMouseCursorChange;
		}
		else {
			// Let ImGui_ImplGlfw_NewFrame's DisplaySize / FramebufferScale
			// stand. They reflect the live GLFW window so layout reflows
			// at the actual size instead of stretching a fixed FBO blit.
			io.ConfigFlags &= ~ImGuiConfigFlags_NoMouseCursorChange;
		}

		ImGui::NewFrame();

		DrawShellWindow(context, plugins);
		ImGui::Render();

		// Two render paths share one ImGui::GetDrawData() call: the
		// content laid out above is rasterised into whichever FBO the
		// current frame targets. The clear color tracks the theme's
		// WindowBg so a Light theme does not leave a dark gutter around
		// the ImGui rectangle.
		const ImVec4 clearCol = ImGui::GetStyleColorVec4(ImGuiCol_WindowBg);

		int fbw = 0;
		int fbh = 0;
		glfwGetFramebufferSize(window, &fbw, &fbh);

		if (vrSurfaceVisible) {
			// VR path: render into the fixed-size FBO and submit. The
			// desktop blit stretches because monitor users behind a VR
			// session are not the primary audience here.
			glBindFramebuffer(GL_FRAMEBUFFER, vrFbo);
			glViewport(0, 0, kVrFboWidth, kVrFboHeight);
			glClearColor(clearCol.x, clearCol.y, clearCol.z, clearCol.w);
			glClear(GL_COLOR_BUFFER_BIT);
			ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
			glBindFramebuffer(GL_FRAMEBUFFER, 0);

			if (fbw > 0 && fbh > 0) {
				glBindFramebuffer(GL_READ_FRAMEBUFFER, vrFbo);
				glBindFramebuffer(GL_DRAW_FRAMEBUFFER, 0);
				glBlitFramebuffer(0, 0, kVrFboWidth, kVrFboHeight, 0, 0, fbw, fbh, GL_COLOR_BUFFER_BIT, GL_LINEAR);
				glBindFramebuffer(GL_READ_FRAMEBUFFER, 0);
				glfwSwapBuffers(window);
			}

			vrOverlay->SubmitTexture(vrTexture, kVrFboWidth, kVrFboHeight);
		}
		else if (fbw > 0 && fbh > 0) {
			// Desktop path: keep the window FBO sized to the actual
			// framebuffer so the 1:1 blit below preserves pixel sharpness
			// and ImGui's layout (already taken at GLFW's DisplaySize)
			// covers the visible area exactly.
			if (fbw != curWinFbW || fbh != curWinFbH) {
				reallocWinFbo(fbw, fbh);
			}

			glBindFramebuffer(GL_FRAMEBUFFER, winFbo);
			glViewport(0, 0, fbw, fbh);
			glClearColor(clearCol.x, clearCol.y, clearCol.z, clearCol.w);
			glClear(GL_COLOR_BUFFER_BIT);
			ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
			glBindFramebuffer(GL_FRAMEBUFFER, 0);

			glBindFramebuffer(GL_READ_FRAMEBUFFER, winFbo);
			glBindFramebuffer(GL_DRAW_FRAMEBUFFER, 0);
			glBlitFramebuffer(0, 0, fbw, fbh, 0, 0, fbw, fbh, GL_COLOR_BUFFER_BIT, GL_NEAREST);
			glBindFramebuffer(GL_READ_FRAMEBUFFER, 0);
			glfwSwapBuffers(window);
		}

		// Wait for input or a frame interval. Tighter cadence when
		// the in-VR overlay is visible so the dashboard stays
		// responsive; broader cadence otherwise lets the desktop
		// process idle cheaply.
		constexpr double kDashboardFrameSeconds = 1.0 / 90.0;
		constexpr double kIdleFrameSeconds = 1.0 / 30.0;
		const double waitSeconds = vrSurfaceVisible ? kDashboardFrameSeconds : kIdleFrameSeconds;
		glfwWaitEventsTimeout(waitSeconds);
	}

	openvr_pair::common::WriteRuntimeHealthSummary();
	const bool steamVrQuitRequested = vrOverlay->QuitRequested();
	vrOverlay.reset();

	for (auto it = plugins.rbegin(); it != plugins.rend(); ++it) {
		openvr_pair::common::DiagnosticLog("overlay", "plugin_shutdown name='%s'", (*it)->Name());
		(*it)->OnShutdown(context);
	}
	if (steamVrQuitRequested) {
		const UpdateInstallState install = openvr_pair::overlay::GetUpdateNoticeState().install;
		if (install.queuedForSteamVrExit && install.phase == UpdateInstallPhase::Ready) {
			std::string updateLaunchError;
			if (!openvr_pair::overlay::LaunchQueuedUpdateAfterProcessExit(GetCurrentProcessId(), &updateLaunchError)) {
				openvr_pair::common::DiagnosticLog("updater", "launch_after_steamvr_close_failed error='%s'",
				                                   updateLaunchError.c_str());
			}
		}
	}
	openvr_pair::common::DiagnosticLog("overlay", "shutdown");

	glDeleteFramebuffers(1, &vrFbo);
	glDeleteTextures(1, &vrTexture);
	glDeleteFramebuffers(1, &winFbo);
	glDeleteTextures(1, &winTexture);

	ImGui_ImplOpenGL3_Shutdown();
	ImGui_ImplGlfw_Shutdown();
	ImPlot::DestroyContext();
	ImGui::DestroyContext();
	glfwDestroyWindow(window);
	glfwTerminate();
	return 0;
}
