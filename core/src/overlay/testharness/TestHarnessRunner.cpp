#include "TestHarnessRunner.h"

#if WKOPENVR_BUILD_IS_DEV

#include "BuildChannel.h"
#include "HarnessScenario.h"
#include "InProcessDriverLoader.h"
#include "MockPoseSource.h"
#include "SandboxStaging.h"
#include "SteamVRConflictGuard.h"

#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <windows.h>

#include <chrono>
#include <cstdio>
#include <cstring>
#include <exception>
#include <cstdlib>
#include <filesystem>
#include <set>
#include <string>
#include <string_view>
#include <vector>

namespace openvr_pair::overlay::testharness {

namespace {

struct Options
{
	std::set<std::string> filter; // empty = all
	bool keep_sandbox = false;
	bool verbose = false;
	std::filesystem::path phantom_replay_path;
	std::filesystem::path phantom_replay_report_path;
	std::string phantom_replay_dropout_role;
	double phantom_replay_dropout_start_ms = -1.0;
	double phantom_replay_dropout_end_ms = -1.0;
	double phantom_replay_speed = 1.0;
};

Options ParseArgs(int argc, char** argv)
{
	Options o;
	for (int i = 1; i < argc; ++i) {
		const std::string_view arg(argv[i]);
		if (arg == "--test-harness") continue;
		if (arg == "--keep-sandbox") {
			o.keep_sandbox = true;
			continue;
		}
		if (arg == "--verbose") {
			o.verbose = true;
			continue;
		}
		if (arg.substr(0, 9) == "--filter=") {
			std::string list(arg.substr(9));
			while (!list.empty()) {
				auto comma = list.find(',');
				std::string slug = (comma == std::string::npos) ? list : list.substr(0, comma);
				if (!slug.empty()) o.filter.insert(slug);
				if (comma == std::string::npos) break;
				list.erase(0, comma + 1);
			}
			continue;
		}
		if (arg == "--phantom-replay" && i + 1 < argc) {
			o.phantom_replay_path = argv[++i];
			continue;
		}
		if (arg.substr(0, 17) == "--phantom-replay=") {
			o.phantom_replay_path = std::string(arg.substr(17));
			continue;
		}
		if (arg == "--phantom-replay-report" && i + 1 < argc) {
			o.phantom_replay_report_path = argv[++i];
			continue;
		}
		if (arg.substr(0, 24) == "--phantom-replay-report=") {
			o.phantom_replay_report_path = std::string(arg.substr(24));
			continue;
		}
		if (arg == "--phantom-replay-dropout-role" && i + 1 < argc) {
			o.phantom_replay_dropout_role = argv[++i];
			continue;
		}
		if (arg.substr(0, 30) == "--phantom-replay-dropout-role=") {
			o.phantom_replay_dropout_role = std::string(arg.substr(30));
			continue;
		}
		if (arg == "--phantom-replay-dropout-start-ms" && i + 1 < argc) {
			o.phantom_replay_dropout_start_ms = std::strtod(argv[++i], nullptr);
			continue;
		}
		if (arg.substr(0, 34) == "--phantom-replay-dropout-start-ms=") {
			o.phantom_replay_dropout_start_ms = std::strtod(std::string(arg.substr(34)).c_str(), nullptr);
			continue;
		}
		if (arg == "--phantom-replay-dropout-end-ms" && i + 1 < argc) {
			o.phantom_replay_dropout_end_ms = std::strtod(argv[++i], nullptr);
			continue;
		}
		if (arg.substr(0, 32) == "--phantom-replay-dropout-end-ms=") {
			o.phantom_replay_dropout_end_ms = std::strtod(std::string(arg.substr(32)).c_str(), nullptr);
			continue;
		}
		if (arg == "--phantom-replay-speed" && i + 1 < argc) {
			o.phantom_replay_speed = std::strtod(argv[++i], nullptr);
			continue;
		}
		if (arg.substr(0, 23) == "--phantom-replay-speed=") {
			o.phantom_replay_speed = std::strtod(std::string(arg.substr(23)).c_str(), nullptr);
			continue;
		}
		// --filter <slug>
		if (arg == "--filter" && i + 1 < argc) {
			std::string list(argv[++i]);
			while (!list.empty()) {
				auto comma = list.find(',');
				std::string slug = (comma == std::string::npos) ? list : list.substr(0, comma);
				if (!slug.empty()) o.filter.insert(slug);
				if (comma == std::string::npos) break;
				list.erase(0, comma + 1);
			}
			continue;
		}
		std::fprintf(stderr, "[testharness] unknown flag '%.*s' (ignored)\n", (int)arg.size(), arg.data());
	}
	if (!o.phantom_replay_path.empty() && o.filter.empty()) {
		o.filter.insert("phantom_replay");
	}
	if (o.phantom_replay_speed <= 0.0) {
		o.phantom_replay_speed = 1.0;
	}
	return o;
}

bool ShouldRun(const std::set<std::string>& filter, const std::string& slug)
{
	if (filter.empty()) return true;
	return filter.count(slug) != 0;
}

class ScopedEnvironmentVariable
{
public:
	ScopedEnvironmentVariable(const wchar_t* name, const std::wstring& value) : name_(name)
	{
		const DWORD len = GetEnvironmentVariableW(name_.c_str(), nullptr, 0);
		if (len != 0) {
			had_previous_ = true;
			previous_.assign(len, L'\0');
			const DWORD written = GetEnvironmentVariableW(name_.c_str(), previous_.data(), len);
			if (written != 0 && written < len) {
				previous_.resize(written);
			}
			else {
				previous_.clear();
				had_previous_ = false;
			}
		}
		SetEnvironmentVariableW(name_.c_str(), value.c_str());
	}

	~ScopedEnvironmentVariable()
	{
		SetEnvironmentVariableW(name_.c_str(), had_previous_ ? previous_.c_str() : nullptr);
	}

	ScopedEnvironmentVariable(const ScopedEnvironmentVariable&) = delete;
	ScopedEnvironmentVariable& operator=(const ScopedEnvironmentVariable&) = delete;

private:
	std::wstring name_;
	std::wstring previous_;
	bool had_previous_ = false;
};

} // namespace

int Run(int argc, char** argv)
{
	if (std::strcmp(WKOPENVR_BUILD_CHANNEL, "dev") != 0) {
		std::fprintf(stderr, "--test-harness is only valid on dev builds (channel=%s)\n", WKOPENVR_BUILD_CHANNEL);
		return 2;
	}

	const Options opts = ParseArgs(argc, argv);

	std::fprintf(stdout, "WKOpenVR test harness (build %s, channel %s)\n", WKOPENVR_BUILD_STAMP,
	             WKOPENVR_BUILD_CHANNEL);
	std::fflush(stdout);

	// 1. Refuse if SteamVR is alive or our shmem is squatting somewhere.
	{
		const auto check = CheckSteamVRConflicts();
		if (!check.ok) {
			std::fprintf(stderr, "[testharness] %s\n", check.error.c_str());
			return 3;
		}
	}

	// 2. Stage sandbox.
	SandboxStaging sandbox;
	if (opts.keep_sandbox) sandbox.keep_on_destruct();
	SandboxLayout layout;
	try {
		StageOptions so;
		so.features = {"calibration",  "smoothing", "dashboardinput", "inputhealth",
		               "facetracking", "oscrouter", "captions",       "phantom"};
		so.include_facetracking_host = true;
		so.include_captions_host = true;
		so.include_phantom_sidecar = true;
		layout = sandbox.Stage(so);
	}
	catch (const std::exception& ex) {
		std::fprintf(stderr, "[testharness] sandbox staging failed: %s\n", ex.what());
		return 4;
	}
	std::fprintf(stdout, "[testharness] sandbox staged at %s\n", layout.root.string().c_str());

	const ScopedEnvironmentVariable moduleSafetyRoot(L"WKOPENVR_MODULE_SAFETY_ROOT",
	                                                 (layout.root / L"module_safety").wstring());

	// 3. Mock runtime + driver loader.
	MockOpenVRRuntime mock(layout.driver_resources);
	MockPoseSource pose_source(mock);
	InProcessDriverLoader loader;
	try {
		loader.Load(layout.driver_dll, mock);
	}
	catch (const std::exception& ex) {
		std::fprintf(stderr, "[testharness] driver load failed: %s\n", ex.what());
		return 5;
	}
	loader.StartFrameTicker();
	std::fprintf(stdout, "[testharness] driver loaded; ticker running\n");
	std::fflush(stdout);

	// 4. Scenarios.
	struct Entry
	{
		const char* slug;
		ScenarioFn run;
	};
	const Entry kScenarios[] = {
	    {"calibration", &RunScenario_calibration},
	    {"smoothing", &RunScenario_smoothing},
	    {"dashboardinput", &RunScenario_dashboardinput},
	    {"inputhealth", &RunScenario_inputhealth},
	    {"facetracking", &RunScenario_facetracking},
	    {"oscrouter", &RunScenario_oscrouter},
	    {"captions", &RunScenario_captions},
	    {"phantom", &RunScenario_phantom},
	    {"phantom_replay", &RunScenario_phantom_replay},
	};

	std::vector<ScenarioResult> results;
	for (const auto& entry : kScenarios) {
		if (std::strcmp(entry.slug, "phantom_replay") == 0 && opts.phantom_replay_path.empty() && opts.filter.empty()) {
			continue;
		}
		if (!ShouldRun(opts.filter, entry.slug)) continue;

		HarnessLogger log(entry.slug);
		log.Step("starting scenario");

		ScenarioContext ctx{
		    mock, loader.provider(), pose_source, layout.root, layout.driver_root, layout.driver_resources, log,
		};
		ctx.phantom_replay_path = opts.phantom_replay_path;
		ctx.phantom_replay_report_path = opts.phantom_replay_report_path;
		ctx.phantom_replay_dropout_role = opts.phantom_replay_dropout_role;
		ctx.phantom_replay_dropout_start_ms = opts.phantom_replay_dropout_start_ms;
		ctx.phantom_replay_dropout_end_ms = opts.phantom_replay_dropout_end_ms;
		ctx.phantom_replay_speed = opts.phantom_replay_speed;

		const auto t0 = std::chrono::steady_clock::now();
		ScenarioResult result;
		try {
			result = entry.run(ctx);
		}
		catch (const std::exception& ex) {
			const auto dt =
			    std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - t0);
			result = Fail(entry.slug, dt, std::string("scenario threw std::exception: ") + ex.what());
		}
		catch (...) {
			const auto dt =
			    std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - t0);
			result = Fail(entry.slug, dt, "scenario threw an unknown exception");
		}
		if (result.name.empty()) result.name = entry.slug;
		results.push_back(result);

		if (result.passed) {
			log.Info("PASS (" + std::to_string(result.duration.count()) + " ms)");
		}
		else {
			log.Error("FAIL: " + result.failure_reason);
		}
	}

	// 5. Shutdown.
	std::fprintf(stdout, "[testharness] stopping driver\n");
	std::fflush(stdout);
	loader.Stop();

	// 6. Report.
	std::fprintf(stdout, "\n=== test harness summary ===\n");
	size_t passed = 0;
	for (const auto& r : results) {
		std::fprintf(stdout, "  %-14s %s  (%lld ms)\n", r.name.c_str(), r.passed ? "PASS" : "FAIL",
		             (long long)r.duration.count());
		if (!r.passed) std::fprintf(stdout, "        %s\n", r.failure_reason.c_str());
		if (r.passed) ++passed;
	}
	std::fprintf(stdout, "%zu / %zu scenarios passed\n", passed, results.size());
	std::fflush(stdout);

	if (results.empty()) {
		std::fprintf(stderr, "[testharness] no scenarios ran (filter excluded all)\n");
		return 6;
	}
	return passed == results.size() ? 0 : 1;
}

} // namespace openvr_pair::overlay::testharness

#endif // WKOPENVR_BUILD_IS_DEV
