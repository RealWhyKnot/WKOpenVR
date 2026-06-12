#include "ShellUi.h"

#include "BugReportUi.h"
#include "DebugLogging.h"
#include "FeaturePlugin.h"
#include "ModuleToggleUi.h"
#include "PerfStatsHub.h"
#include "ShellContext.h"
#include "ShellUiLogic.h"
#include "Theme.h"
#include "UiCore.h"
#include "UpdateNotice.h"

#include <imgui.h>
#include <implot.h>

#ifdef _WIN32
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <windows.h>
#include <shellapi.h>
#endif

#include <algorithm>
#include <cstdint>
#include <cstdio>
#include <string>
#include <vector>

namespace openvr_pair::overlay {

namespace {

namespace module_registry = openvr_pair::common::modules;

void DrawTransientStatus(ShellContext& context)
{
	if (context.status.empty()) return;
	ImGui::Separator();
	if (ImGui::SmallButton("x##dismiss_shell_status")) {
		context.ClearStatus();
		return;
	}
	if (ImGui::IsItemHovered()) {
		ImGui::SetTooltip("Dismiss status");
	}
	ImGui::SameLine();
	ui::DrawTextWrapped(context.status.c_str());
}

FeaturePlugin* FindDefaultLogsPanelPlugin(std::vector<std::unique_ptr<FeaturePlugin>>& plugins)
{
	for (auto& plugin : plugins) {
		if (plugin && IsDefaultLogsPanelPlugin(plugin->FlagFileName())) {
			return plugin.get();
		}
	}
	return nullptr;
}

std::string FormatUpdateDownloadProgress(const UpdateInstallState& install)
{
	if (install.totalBytes <= 0) return {};
	const double pct =
	    install.bytesDownloaded <= 0
	        ? 0.0
	        : (100.0 * static_cast<double>(install.bytesDownloaded) / static_cast<double>(install.totalBytes));
	char buf[160] = {};
	snprintf(buf, sizeof(buf), "%s of %s (%.0f%%)", ui::FormatByteCount(install.bytesDownloaded).c_str(),
	         ui::FormatByteCount(install.totalBytes).c_str(), pct);
	return buf;
}

ImPlotPoint PerfSeriesGetter(int idx, void* ptr)
{
	const auto* series = static_cast<const PerfTimeSeries*>(ptr);
	const auto& point = series->At(static_cast<size_t>(idx));
	return ImPlotPoint(point.first, point.second);
}

std::string FormatPercent(double value)
{
	char buf[32] = {};
	snprintf(buf, sizeof(buf), "%.2f%%", value);
	return buf;
}

std::string FormatPid(uint32_t pid)
{
	if (pid == 0) return "-";
	char buf[32] = {};
	snprintf(buf, sizeof(buf), "%lu", static_cast<unsigned long>(pid));
	return buf;
}

std::string FormatThreadTriple(uint32_t overlayThreads, uint32_t driverThreads, uint32_t sidecarThreads)
{
	char buf[64] = {};
	snprintf(buf, sizeof(buf), "%lu / %lu / %lu", static_cast<unsigned long>(overlayThreads),
	         static_cast<unsigned long>(driverThreads), static_cast<unsigned long>(sidecarThreads));
	return buf;
}

ui::StatusTone PerfTone(double pctOneCore)
{
	if (pctOneCore >= 50.0) return ui::StatusTone::Error;
	if (pctOneCore >= 20.0) return ui::StatusTone::Warn;
	if (pctOneCore > 0.0) return ui::StatusTone::Info;
	return ui::StatusTone::Idle;
}

double LatestPerfTimestamp(const PerfHistoryStore& history)
{
	double latest = 0.0;
	auto consider = [&latest](const PerfTimeSeries& series) {
		if (!series.Empty()) latest = std::max(latest, series.At(series.Size() - 1).first);
	};
	consider(history.overlay.totalCpuPctOneCore);
	consider(history.driver.totalCpuPctOneCore);
	consider(history.sidecarTotalCpuPctOneCore);
	return latest;
}

float MaxCpuSeriesValue(const PerfHistoryStore& history)
{
	float best = 0.0f;
	best = std::max(best, history.overlay.totalCpuPctOneCore.Max());
	best = std::max(best, history.driver.totalCpuPctOneCore.Max());
	best = std::max(best, history.sidecarTotalCpuPctOneCore.Max());
	best = std::max(best, history.overlay.unattributedPctOneCore.Max());
	best = std::max(best, history.driver.unattributedPctOneCore.Max());
	return best;
}

void PlotPerfSeries(const char* label, const PerfTimeSeries& series)
{
	if (series.Empty()) return;
	ImPlot::PlotLineG(label, PerfSeriesGetter, const_cast<PerfTimeSeries*>(&series), static_cast<int>(series.Size()));
}

void DrawPerfCpuGraph(const PerfHistoryStore& history)
{
	const double latest = LatestPerfTimestamp(history);
	if (latest <= 0.0) {
		ui::DrawEmptyState("Waiting for performance samples.");
		return;
	}

	const float yMax = std::max(10.0f, MaxCpuSeriesValue(history) * 1.20f);
	if (ImPlot::BeginPlot("##module_perf_cpu", ImVec2(-1.0f, 190.0f), ImPlotFlags_NoMenus)) {
		ImPlot::SetupAxes("seconds", "% of one core", ImPlotAxisFlags_NoTickLabels, 0);
		ImPlot::SetupAxisLimits(ImAxis_X1, latest - PerfTimeSeries::kWindowSeconds, latest, ImGuiCond_Always);
		ImPlot::SetupAxisLimits(ImAxis_Y1, 0.0, yMax, ImGuiCond_Always);
		PlotPerfSeries("Overlay", history.overlay.totalCpuPctOneCore);
		PlotPerfSeries("Driver host", history.driver.totalCpuPctOneCore);
		PlotPerfSeries("Sidecars", history.sidecarTotalCpuPctOneCore);
		PlotPerfSeries("Overlay other", history.overlay.unattributedPctOneCore);
		PlotPerfSeries("Driver other", history.driver.unattributedPctOneCore);
		ImPlot::EndPlot();
	}
}

void DrawPerfMemoryGraph(const PerfHistoryStore& history)
{
	const double latest = LatestPerfTimestamp(history);
	if (latest <= 0.0) return;
	const float yMax =
	    std::max(64.0f, std::max(history.overlay.workingSetMb.Max(), history.driver.workingSetMb.Max()) * 1.15f);
	if (ImPlot::BeginPlot("##module_perf_memory", ImVec2(-1.0f, 130.0f), ImPlotFlags_NoMenus)) {
		ImPlot::SetupAxes("seconds", "working set MB", ImPlotAxisFlags_NoTickLabels, 0);
		ImPlot::SetupAxisLimits(ImAxis_X1, latest - PerfTimeSeries::kWindowSeconds, latest, ImGuiCond_Always);
		ImPlot::SetupAxisLimits(ImAxis_Y1, 0.0, yMax, ImGuiCond_Always);
		PlotPerfSeries("Overlay", history.overlay.workingSetMb);
		PlotPerfSeries("Driver host", history.driver.workingSetMb);
		ImPlot::EndPlot();
	}
}

void DrawPerfProcessRow(const char* label, uint32_t pid, double cpuPct, double otherCpuPct, double peakPct,
                        double p95Pct, double memoryMb, uint32_t threads, uint32_t handles, const char* status)
{
	ui::NextRow();
	ui::NextColumn();
	ImGui::TextUnformatted(label);
	ui::NextColumn();
	ImGui::TextUnformatted(FormatPid(pid).c_str());
	ui::NextColumn();
	ui::DrawStatusCell(FormatPercent(cpuPct).c_str(), PerfTone(cpuPct), true);
	ui::NextColumn();
	ui::DrawStatusCell(FormatPercent(otherCpuPct).c_str(), PerfTone(otherCpuPct), true);
	ui::NextColumn();
	// Window peak next to the live value makes a transient spike visible even
	// after the smoothed reading has settled. The p95 (spike-tolerant "busy"
	// level) rides along in a tooltip.
	ui::DrawStatusCell(FormatPercent(peakPct).c_str(), PerfTone(peakPct), true);
	if (ImGui::IsItemHovered()) {
		ImGui::SetTooltip("120s peak %s, p95 %s", FormatPercent(peakPct).c_str(), FormatPercent(p95Pct).c_str());
	}
	ui::NextColumn();
	ImGui::Text("%.1f", memoryMb);
	ui::NextColumn();
	ImGui::Text("%lu", static_cast<unsigned long>(threads));
	ui::NextColumn();
	ImGui::Text("%lu", static_cast<unsigned long>(handles));
	ui::NextColumn();
	ImGui::TextUnformatted(status);
}

void DrawPerfProcessTable(const PerfViewModel& vm, const PerfHistoryStore& history)
{
	ui::TableScope table("module_perf_processes", 9,
	                     ImGuiTableFlags_BordersInnerH | ImGuiTableFlags_RowBg | ImGuiTableFlags_SizingStretchProp);
	if (!table) return;
	ui::SetupStretchColumn("Process", 1.0f);
	ui::SetupFixedColumn("PID", 70.0f);
	ui::SetupFixedColumn("CPU", 76.0f);
	ui::SetupFixedColumn("Other", 76.0f);
	ui::SetupFixedColumn("Peak", 76.0f);
	ui::SetupFixedColumn("WS MB", 80.0f);
	ui::SetupFixedColumn("Threads", 82.0f);
	ui::SetupFixedColumn("Handles", 82.0f);
	ui::SetupFixedColumn("Status", 110.0f);
	ui::DrawTableHeader();

	const PerfSpikeStats overlaySpikes = ComputeSpikeStats(history.overlay.totalCpuPctOneCore);
	DrawPerfProcessRow("Overlay", vm.overlayPid, vm.overlayTotalPct, vm.overlayUnattributedPct, overlaySpikes.peak,
	                   overlaySpikes.p95, vm.overlayWorkingSetMb, vm.overlayThreadCount, vm.overlayHandleCount,
	                   "local");
	if (vm.driverConnected) {
		const PerfSpikeStats driverSpikes = ComputeSpikeStats(history.driver.totalCpuPctOneCore);
		DrawPerfProcessRow("Driver host", vm.driverPid, vm.driverTotalPct, vm.driverUnattributedPct, driverSpikes.peak,
		                   driverSpikes.p95, vm.driverWorkingSetMb, vm.driverThreadCount, vm.driverHandleCount, "live");
	}
	else {
		DrawPerfProcessRow("Driver host", 0, 0.0, 0.0, 0.0, 0.0, 0.0, 0, 0, "not mapped");
	}
}

void DrawPerfModuleTable(const PerfViewModel& vm, const PerfStatsHub& hub, const PerfHistoryStore& history)
{
	if (vm.rows.empty()) {
		ui::DrawEmptyState("No module samples yet.");
		return;
	}

	ui::TableScope table("module_perf_attribution", 8,
	                     ImGuiTableFlags_BordersInnerH | ImGuiTableFlags_RowBg | ImGuiTableFlags_SizingStretchProp);
	if (!table) return;
	ui::SetupStretchColumn("Module", 1.0f);
	ui::SetupFixedColumn("Total", 76.0f);
	ui::SetupFixedColumn("Peak", 70.0f);
	ui::SetupFixedColumn("Overlay", 76.0f);
	ui::SetupFixedColumn("Driver", 76.0f);
	ui::SetupFixedColumn("Sidecar", 82.0f);
	ui::SetupFixedColumn("Threads", 100.0f);
	ui::SetupFixedColumn("Sidecar WS", 96.0f);
	ui::DrawTableHeader();

	for (const PerfModuleRow& row : vm.rows) {
		const uint32_t slot = openvr_pair::common::moduleperf::SlotIndex(row.id);
		const double smoothedTotal = hub.SmoothedTotalPct(slot);
		// Worst single-process spike this module caused in the window. Per-
		// process peaks do not sum, so report the largest of the three rather
		// than an inflated total.
		const double modulePeak =
		    std::max({history.overlay.moduleCpuPctOneCore[slot].Max(), history.driver.moduleCpuPctOneCore[slot].Max(),
		              history.sidecarModuleCpuPctOneCore[slot].Max()});
		ui::NextRow();
		ui::NextColumn();
		ImGui::TextUnformatted(module_registry::DisplayName(row.id));
		if (row.sidecarPresent && row.sidecarProcessCount > 1) {
			ImGui::SameLine();
			ImGui::TextDisabled("(%lu sidecars)", static_cast<unsigned long>(row.sidecarProcessCount));
		}
		ui::NextColumn();
		ui::DrawStatusCell(FormatPercent(smoothedTotal).c_str(), PerfTone(smoothedTotal), true);
		ui::NextColumn();
		ImGui::TextUnformatted(FormatPercent(modulePeak).c_str());
		ui::NextColumn();
		ImGui::TextUnformatted(FormatPercent(row.overlayPct).c_str());
		ui::NextColumn();
		ImGui::TextUnformatted(row.driverActive ? FormatPercent(row.driverPct).c_str() : "-");
		ui::NextColumn();
		ImGui::TextUnformatted(row.sidecarPresent ? FormatPercent(row.sidecarPct).c_str() : "-");
		ui::NextColumn();
		ImGui::TextUnformatted(FormatThreadTriple(row.overlayThreads, row.driverThreads, row.sidecarThreads).c_str());
		ui::NextColumn();
		if (row.sidecarWorkingSetBytes > 0) {
			ImGui::TextUnformatted(ui::FormatByteCount(row.sidecarWorkingSetBytes).c_str());
		}
		else {
			ImGui::TextUnformatted("-");
		}
		if (row.sidecarPresent && row.sidecarPid != 0 && ImGui::IsItemHovered()) {
			ImGui::SetTooltip("Sidecar PID %lu", static_cast<unsigned long>(row.sidecarPid));
		}
	}
}

void DrawPerformanceCard()
{
	const PerfStatsHub& hub = GetPerfStatsHub();
	const PerfViewModel& vm = hub.ViewModel();
	const PerfHistoryStore& history = hub.History();
	const ui::StatusTone tone = vm.driverConnected ? ui::StatusTone::Info : ui::StatusTone::Warn;

	ui::DrawCard("Performance", tone, [&] {
		if (vm.driverConnected) {
			ui::StatusBadge("Driver live", ui::StatusTone::Ok);
			ImGui::SameLine();
			ImGui::TextDisabled("age %.1fs", vm.driverSnapshotAgeSec);
		}
		else {
			ui::StatusBadge(hub.DriverSegmentOpen() ? "Driver stale" : "Driver offline", ui::StatusTone::Warn);
		}

		ImGui::Spacing();
		DrawPerfProcessTable(vm, history);
		ImGui::Spacing();
		DrawPerfCpuGraph(history);
		DrawPerfMemoryGraph(history);
		ImGui::Spacing();
		DrawPerfModuleTable(vm, hub, history);
	});
}

void DrawUpdatePrompt(ShellContext& context, const std::vector<std::string_view>& installedFlags)
{
	const UpdateNoticeState notice = GetUpdateNoticeState();
	const UpdateInstallState& install = notice.install;
	const bool showPrompt =
	    notice.available || install.queuedForSteamVrExit || install.phase == UpdateInstallPhase::Failed;
	if (!showPrompt) return;

	ui::StatusTone tone = ui::StatusTone::Info;
	if (install.phase == UpdateInstallPhase::Failed) tone = ui::StatusTone::Error;
	if (install.phase == UpdateInstallPhase::Downloading) tone = ui::StatusTone::Warn;
	if (install.phase == UpdateInstallPhase::Ready || install.phase == UpdateInstallPhase::Launching) {
		tone = ui::StatusTone::Ok;
	}

	ui::DrawCard("Update pending", tone, [&] {
		const std::string version = !notice.latestVersion.empty() ? notice.latestVersion : install.targetVersion;
		if (!version.empty()) {
			ImGui::TextUnformatted(("Version v" + version).c_str());
		}

		if (install.phase == UpdateInstallPhase::Downloading) {
			std::string progress = FormatUpdateDownloadProgress(install);
			ui::DrawTextWrapped(progress.empty() ? "Downloading and checking installer..."
			                                     : ("Downloading and checking installer: " + progress).c_str());
		}
		else if (install.phase == UpdateInstallPhase::Ready) {
			ui::DrawTextWrapped("Installer verified. It will start after SteamVR closes.");
		}
		else if (install.phase == UpdateInstallPhase::Launching) {
			ui::DrawTextWrapped("Installer will open after WKOpenVR exits.");
		}
		else if (install.phase == UpdateInstallPhase::Failed) {
			ui::DrawTextWrapped(install.errorMessage.empty() ? "Update download failed."
			                                                 : install.errorMessage.c_str());
		}
		else {
			ui::DrawTextWrapped("Queue the installer now. It will run after SteamVR closes.");
		}

		ImGui::Spacing();
		if (install.phase == UpdateInstallPhase::Idle || install.phase == UpdateInstallPhase::Failed) {
			if (ImGui::Button("Queue update")) {
				std::string error;
				if (!QueueUpdateForSteamVrClose(installedFlags, &error)) {
					context.SetStatus(error.empty() ? "Update was not queued." : error);
				}
				else {
					context.SetStatus("Update queued for the next SteamVR close.");
				}
			}
			if (ImGui::IsItemHovered()) {
				ImGui::SetTooltip("Download, verify, and queue the installer.");
			}
		}
		else {
			if (ImGui::Button("Cancel update")) {
				CancelQueuedUpdate();
				context.SetStatus("Update queue cancelled.");
			}
			if (ImGui::IsItemHovered()) {
				ImGui::SetTooltip("Cancel the queued installer.");
			}
		}

		if (!notice.releaseUrl.empty()) {
			ImGui::SameLine();
			if (ImGui::Button("Release notes")) {
#ifdef _WIN32
				ShellExecuteA(nullptr, "open", notice.releaseUrl.c_str(), nullptr, nullptr, SW_SHOWNORMAL);
#endif
			}
			if (ImGui::IsItemHovered()) {
				ImGui::SetTooltip("%s", notice.releaseUrl.c_str());
			}
		}
	});
	ImGui::Spacing();
}

FeaturePlugin* FindFeatureByFlag(const std::vector<FeaturePlugin*>& plugins, std::string_view flag)
{
	for (FeaturePlugin* plugin : plugins) {
		if (plugin && flag == plugin->FlagFileName()) return plugin;
	}
	return nullptr;
}

enum class ShellBuiltInTab
{
	None,
	Logs,
	Modules,
	Themes,
};

struct ShellTabEntry
{
	std::string key;
	const char* label = nullptr;
	FeaturePlugin* plugin = nullptr;
	ShellBuiltInTab builtIn = ShellBuiltInTab::None;
};

constexpr std::string_view kModuleTabPrefix = "module:";
constexpr const char* kLogsTabKey = "shell:logs";
constexpr const char* kModulesTabKey = "shell:modules";
constexpr const char* kThemesTabKey = "shell:themes";

std::string ModuleTabKey(std::string_view flag)
{
	return std::string(kModuleTabPrefix) + std::string(flag);
}

std::string FeatureTabKey(const FeaturePlugin& plugin)
{
	const char* flag = plugin.FlagFileName();
	if (flag && IsValidModuleFlagForShellOrder(flag)) return ModuleTabKey(flag);
	return std::string("plugin:") + (plugin.Name() ? plugin.Name() : "");
}

std::vector<std::string_view> ModuleFlagsFromPlugins(const std::vector<FeaturePlugin*>& plugins)
{
	std::vector<std::string_view> flags;
	for (FeaturePlugin* plugin : plugins) {
		if (!plugin) continue;
		const char* flag = plugin->FlagFileName();
		if (flag && IsValidModuleFlagForShellOrder(flag)) {
			flags.push_back(flag);
		}
	}
	return flags;
}

std::vector<FeaturePlugin*> OrderPluginsByModuleTabOrder(const std::vector<FeaturePlugin*>& plugins,
                                                         const std::vector<std::string>& preferredOrder)
{
	std::vector<FeaturePlugin*> ordered;
	const std::vector<std::string> resolvedOrder =
	    ResolveModuleTabOrder(preferredOrder, ModuleFlagsFromPlugins(plugins));
	for (const std::string& flag : resolvedOrder) {
		if (FeaturePlugin* plugin = FindFeatureByFlag(plugins, flag)) {
			ordered.push_back(plugin);
		}
	}
	for (FeaturePlugin* plugin : plugins) {
		if (!plugin) continue;
		const char* flag = plugin->FlagFileName();
		if (flag && IsValidModuleFlagForShellOrder(flag)) continue;
		ordered.push_back(plugin);
	}
	return ordered;
}

bool IsFeatureContentTabVisible(ShellContext& context, FeaturePlugin& plugin)
{
	const char* flag = plugin.FlagFileName();
	const bool installed = plugin.IsInstalled(context);
	const bool autoDisabled = context.IsModuleAutoDisabled(flag);
	bool dependencyBlocked = false;

	if (const module_registry::ModuleInfo* module = module_registry::FindByFlagFileName(flag)) {
		dependencyBlocked =
		    module->requires_osc_router &&
		    context.IsModuleAutoDisabled(module_registry::FlagFileName(module_registry::ModuleId::OscRouter));
	}

	return ShouldShowFeatureContentTab(installed, autoDisabled, dependencyBlocked);
}

bool ContainsShellTabKey(const std::vector<ShellTabEntry>& entries, std::string_view key)
{
	for (const ShellTabEntry& entry : entries) {
		if (entry.key == key) return true;
	}
	return false;
}

int ShellTabIndex(const std::vector<ShellTabEntry>& entries, std::string_view key)
{
	for (size_t i = 0; i < entries.size(); ++i) {
		if (entries[i].key == key) return static_cast<int>(i);
	}
	return -1;
}

void MoveShellTabSelection(const std::vector<ShellTabEntry>& entries, std::string& selectedKey, int offset)
{
	if (entries.empty() || offset == 0) return;
	int index = ShellTabIndex(entries, selectedKey);
	if (index < 0) index = 0;
	const int target = std::clamp(index + offset, 0, static_cast<int>(entries.size()) - 1);
	selectedKey = entries[static_cast<size_t>(target)].key;
}

std::vector<ShellTabEntry> BuildShellTabEntries(const std::vector<FeaturePlugin*>& installedPlugins)
{
	std::vector<ShellTabEntry> entries;
	entries.reserve(installedPlugins.size() + 3);
	for (FeaturePlugin* plugin : installedPlugins) {
		if (!plugin) continue;
		entries.push_back({FeatureTabKey(*plugin), plugin->Name(), plugin, ShellBuiltInTab::None});
	}
	entries.push_back({kLogsTabKey, "Logs", nullptr, ShellBuiltInTab::Logs});
	entries.push_back({kModulesTabKey, "Modules", nullptr, ShellBuiltInTab::Modules});
	entries.push_back({kThemesTabKey, "Themes", nullptr, ShellBuiltInTab::Themes});
	return entries;
}

void DrawFallbackLogsPanel(ShellContext& context)
{
	ui::DrawSectionHeading("Debug logging");
	const bool forced = common::IsDebugLoggingForcedOn();
	bool debugLogging = common::IsDebugLoggingEnabled();
	if (forced) debugLogging = true;
	{
		ui::DisabledSection locked(forced, "Dev builds keep debug logging enabled.");
		if (ui::CheckboxWithTooltip("Enable debug logging", &debugLogging,
		                            "Write WKOpenVR diagnostics to %LocalAppDataLow%\\WKOpenVR\\Logs\\.")) {
			common::SetDebugLoggingEnabled(debugLogging);
		}
		locked.AttachReasonTooltip();
	}
	ImGui::SameLine();
	ImGui::TextDisabled(forced ? "(dev build: always on)" : (debugLogging ? "(on)" : "(off)"));

	ImGui::Spacing();
	ui::DrawSectionHeading("Bug reports");
	DrawBugReportButton(context);
}

void NotifyDebugLoggingChanged(std::vector<std::unique_ptr<FeaturePlugin>>& plugins)
{
	const bool effectiveDebugLogging = common::IsDebugLoggingEnabled();
	for (auto& plugin : plugins) {
		if (plugin) {
			plugin->OnDebugLoggingChanged(effectiveDebugLogging);
		}
	}
}

void ResolveShellTabSelection(ShellContext& context, const std::vector<ShellTabEntry>& entries,
                              const std::vector<std::string_view>& installedFlags, std::string& selectedShellTabKey,
                              std::string& desktopDefaultTabAppliedFor, std::string& pendingTabJumpKey)
{
	const std::string desktopDefaultFlag = context.DesktopDefaultModuleFlagFileName();
	if (!context.vrConnected && !desktopDefaultFlag.empty() && desktopDefaultTabAppliedFor != desktopDefaultFlag &&
	    ContainsFeatureFlag(installedFlags, desktopDefaultFlag)) {
		selectedShellTabKey = ModuleTabKey(desktopDefaultFlag);
		pendingTabJumpKey = selectedShellTabKey;
		desktopDefaultTabAppliedFor = desktopDefaultFlag;
	}
	if (!ContainsShellTabKey(entries, selectedShellTabKey)) {
		selectedShellTabKey = installedFlags.empty() && ContainsShellTabKey(entries, kModulesTabKey)
		                          ? std::string(kModulesTabKey)
		                          : (entries.empty() ? std::string() : entries.front().key);
		pendingTabJumpKey = selectedShellTabKey;
	}
}

const ShellTabEntry* FindShellTabEntry(const std::vector<ShellTabEntry>& entries, std::string_view key)
{
	for (const ShellTabEntry& entry : entries) {
		if (entry.key == key) return &entry;
	}
	return nullptr;
}

void DrawShellTabStrip(const std::vector<ShellTabEntry>& entries, std::string& selectedShellTabKey,
                       std::string& pendingTabJumpKey)
{
	if (entries.empty()) {
		pendingTabJumpKey.clear();
		return;
	}

	const int selectedIndex = ShellTabIndex(entries, selectedShellTabKey);
	const bool canMoveLeft = selectedIndex > 0;
	const ImGuiStyle& style = ImGui::GetStyle();
	const float buttonSize = ImGui::GetFrameHeight();

	// Previous-tab arrow. Replaced by a same-size spacer on the first tab so the
	// control only shows when it can do something and the strip never shifts.
	if (canMoveLeft) {
		if (ImGui::ArrowButton("##shell_tab_left", ImGuiDir_Left)) {
			std::string target = selectedShellTabKey;
			MoveShellTabSelection(entries, target, -1);
			pendingTabJumpKey = target;
		}
		if (ImGui::IsItemHovered()) {
			ImGui::SetTooltip("Previous tab");
		}
	}
	else {
		ImGui::Dummy(ImVec2(buttonSize, buttonSize));
	}

	ImGui::SameLine();
	// Reserve the next-tab button plus a spacing of margin so it never lands on
	// the content clip edge (which would clip it out of view entirely).
	const float stripWidth =
	    std::max(120.0f, ImGui::GetContentRegionAvail().x - buttonSize - style.ItemSpacing.x * 2.0f);
	{
		ui::ChildScope strip("##shell_tab_strip",
		                     ImVec2(stripWidth, ImGui::GetFrameHeightWithSpacing() + style.ItemSpacing.y),
		                     ImGuiChildFlags_None, ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse);
		if (strip) {
			ui::TabBarScope tabs("tabs",
			                     ImGuiTabBarFlags_FittingPolicyScroll | ImGuiTabBarFlags_NoTabListScrollingButtons);
			if (tabs) {
				for (const ShellTabEntry& entry : entries) {
					// SetSelected is applied only on the single frame a jump was
					// requested (arrow button or desktop default). Forcing it every
					// frame would override the user's click and snap the selection
					// back to the tracked tab; ImGui owns the selection otherwise.
					ImGuiTabItemFlags flags = ImGuiTabItemFlags_None;
					if (!pendingTabJumpKey.empty() && entry.key == pendingTabJumpKey) {
						flags |= ImGuiTabItemFlags_SetSelected;
					}
					ui::TabItemScope tab(entry.label, nullptr, flags);
					if (tab) {
						selectedShellTabKey = entry.key;
					}
				}
			}
		}
	}
	// A jump request lives for exactly one strip draw. Clearing it here lets the
	// next-tab arrow below schedule a fresh jump for the following frame.
	pendingTabJumpKey.clear();

	ImGui::SameLine();
	const int rightSelectedIndex = ShellTabIndex(entries, selectedShellTabKey);
	const bool canMoveRight = rightSelectedIndex >= 0 && rightSelectedIndex + 1 < static_cast<int>(entries.size());
	if (canMoveRight) {
		if (ImGui::ArrowButton("##shell_tab_right", ImGuiDir_Right)) {
			std::string target = selectedShellTabKey;
			MoveShellTabSelection(entries, target, 1);
			pendingTabJumpKey = target;
		}
		if (ImGui::IsItemHovered()) {
			ImGui::SetTooltip("Next tab");
		}
	}
	else {
		ImGui::Dummy(ImVec2(buttonSize, buttonSize));
	}
}

} // namespace

void DrawLogsTab(ShellContext& context, std::vector<std::unique_ptr<FeaturePlugin>>& plugins)
{
	if (FeaturePlugin* logsPlugin = FindDefaultLogsPanelPlugin(plugins)) {
		logsPlugin->DrawLogsSection(context);
		ImGui::Spacing();
		ImGui::Separator();
		ImGui::Spacing();
		ui::DrawSectionHeading("Bug reports");
		DrawBugReportButton(context);
	}
	else {
		DrawFallbackLogsPanel(context);
	}
	NotifyDebugLoggingChanged(plugins);
}

void DrawModulesTab(ShellContext& context, std::vector<std::unique_ptr<FeaturePlugin>>& plugins)
{
	ImGui::TextUnformatted("Modules");
	ui::DrawTextWrapped("Toggle features on or off. Each change pops a UAC prompt. "
	                    "Changes take effect the next time SteamVR loads the driver.");
	ImGui::Spacing();

	std::vector<FeaturePlugin*> modules;
	for (auto& plugin : plugins) {
		if (ShouldShowInModulesTab(*plugin)) {
			modules.push_back(plugin.get());
		}
	}
	modules = OrderPluginsByModuleTabOrder(modules, context.ModuleTabOrder());
	ModuleToggleTableOptions options;
	options.markDevelopmentModules = true;
	options.allowTabReorder = true;
	DrawModuleToggleTable(context, modules, "modules", "No modules were compiled into this build.", options);
	ImGui::Spacing();
	DrawPerformanceCard();
}

void DrawThemesTab(ShellContext&)
{
	ui::DrawSectionHeading("Color theme");
	ui::DrawTextWrapped("Choose a color theme. Changes apply immediately and persist across launches.");
	ImGui::Spacing();

	const ui::ThemeId current = ui::GetCurrentThemeId();
	for (int i = 0; i < (int)ui::ThemeId::Count_; ++i) {
		const ui::ThemeId id = (ui::ThemeId)i;
		const bool selected = (id == current);
		ImGui::PushID(i);
		if (ui::RadioButtonWithTooltip(ui::ThemeName(id), selected, ui::ThemeCaption(id))) {
			ui::SetTheme(id);
		}
		ImGui::SameLine();
		ImGui::TextDisabled("%s", ui::ThemeCaption(id));
		ImGui::PopID();
	}
}

void DrawShellWindow(ShellContext& context, std::vector<std::unique_ptr<FeaturePlugin>>& plugins)
{
	static std::string desktopDefaultTabAppliedFor;
	static std::string selectedShellTabKey;
	static std::string pendingTabJumpKey;
	context.TickStatus();

	const ImGuiViewport* vp = ImGui::GetMainViewport();
	ImGui::SetNextWindowPos(vp->WorkPos);
	ImGui::SetNextWindowSize(vp->WorkSize);
	const ImGuiWindowFlags flags = ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove |
	                               ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoBringToFrontOnFocus |
	                               ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse;

	ImGui::Begin("WKOpenVR", nullptr, flags);

	const bool hasStatus = !context.status.empty();
	const float statusReserve =
	    hasStatus ? ImGui::GetTextLineHeightWithSpacing() * 3.0f + ImGui::GetStyle().ItemSpacing.y : 0.0f;

	if (ImGui::BeginChild("##shell_content", ImVec2(0.0f, hasStatus ? -statusReserve : 0.0f), false,
	                      ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse)) {
		std::vector<FeaturePlugin*> installedPlugins;
		for (auto& plugin : plugins) {
			if (!plugin || !IsFeatureContentTabVisible(context, *plugin)) continue;
			installedPlugins.push_back(plugin.get());
		}
		installedPlugins = OrderPluginsByModuleTabOrder(installedPlugins, context.ModuleTabOrder());
		const std::vector<std::string_view> installedFlags = ModuleFlagsFromPlugins(installedPlugins);
		const std::vector<ShellTabEntry> entries = BuildShellTabEntries(installedPlugins);
		ResolveShellTabSelection(context, entries, installedFlags, selectedShellTabKey, desktopDefaultTabAppliedFor,
		                         pendingTabJumpKey);
		DrawUpdatePrompt(context, installedFlags);
		DrawShellTabStrip(entries, selectedShellTabKey, pendingTabJumpKey);

		ImGui::Spacing();
		ui::ChildScope body("##shell_tab_body", ImVec2(0.0f, 0.0f));
		if (body) {
			const ShellTabEntry* active = FindShellTabEntry(entries, selectedShellTabKey);
			if (active && active->plugin) {
				active->plugin->DrawTab(context);
			}
			else if (active && active->builtIn == ShellBuiltInTab::Logs) {
				DrawLogsTab(context, plugins);
			}
			else if (active && active->builtIn == ShellBuiltInTab::Modules) {
				DrawModulesTab(context, plugins);
			}
			else if (active && active->builtIn == ShellBuiltInTab::Themes) {
				DrawThemesTab(context);
			}
		}
	}
	ImGui::EndChild();

	DrawTransientStatus(context);
	ImGui::End();
}

} // namespace openvr_pair::overlay
