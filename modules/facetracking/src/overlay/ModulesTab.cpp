#include "ModulesTab.h"

#include "FacetrackingPlugin.h"
#include "Logging.h"
#include "ModuleSources.h"
#include "UiHelpers.h"
#include "Win32Text.h"

#include "picojson.h"

#include <imgui/imgui.h>

#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#include <shobjidl.h>

#include <algorithm>
#include <string>
#include <vector>

namespace facetracking::ui {

using namespace openvr_pair::overlay::ui;

// ---- state shared across draws (per-overlay-instance, reset on tab show) -

struct ModulesTabState
{
    // Installed-modules list (refreshed each frame from disk scan,
    // throttled so we don't hammer the FS every 60 Hz frame).
    std::vector<InstalledModule>  installed;
    std::vector<AvailableModule>  available;
    SourcesCatalogue              catalogue;
    int64_t                       last_scan_tick = 0;   // ::GetTickCount64()

    // Last sync status string shown near the sources table.
    // Populated by polling plugin.sync_runner_ each draw.
    std::string sync_status;
    bool        sync_status_ok = true;

    // GitHub "add source" popup input.
    char github_input[256] = {};

    bool initialised = false;
};

static ModulesTabState g_state;

static void RefreshIfStale()
{
    uint64_t now = GetTickCount64();
    // Rescan at most once per second.
    if (now - static_cast<uint64_t>(g_state.last_scan_tick) < 1000 && g_state.initialised)
        return;

    g_state.last_scan_tick = static_cast<int64_t>(now);
    g_state.installed      = ScanInstalledModules();
    g_state.catalogue      = EnsureSourcesCatalogue();
    g_state.available      = LoadAvailableModules();
    g_state.initialised    = true;
}

// Open a Win32 folder-picker dialog (IFileOpenDialog + FOS_PICKFOLDERS).
// Returns empty string if the user cancels or COM fails.
static std::string PickFolder()
{
    std::string result;
    IFileOpenDialog *pDlg = nullptr;
    if (FAILED(CoCreateInstance(CLSID_FileOpenDialog, nullptr, CLSCTX_INPROC_SERVER,
                                IID_PPV_ARGS(&pDlg))))
        return {};

    DWORD opts = 0;
    pDlg->GetOptions(&opts);
    pDlg->SetOptions(opts | FOS_PICKFOLDERS | FOS_FORCEFILESYSTEM);
    pDlg->SetTitle(L"Select face-tracking module folder");

    if (SUCCEEDED(pDlg->Show(nullptr))) {
        IShellItem *pItem = nullptr;
        if (SUCCEEDED(pDlg->GetResult(&pItem))) {
            PWSTR path = nullptr;
            if (SUCCEEDED(pItem->GetDisplayName(SIGDN_FILESYSPATH, &path))) {
                result = openvr_pair::common::WideToUtf8(path);
                CoTaskMemFree(path);
            }
            pItem->Release();
        }
    }
    pDlg->Release();
    return result;
}

// Parse "https://github.com/owner/repo" or "owner/repo" -> "owner/repo".
static std::string ParseOwnerRepo(const std::string &input)
{
    static const char prefix[] = "https://github.com/";
    if (input.rfind(prefix, 0) == 0)
        return input.substr(sizeof(prefix) - 1);
    // Rudimentary validation: must contain exactly one '/'.
    size_t slash = input.find('/');
    if (slash == std::string::npos || slash == 0 || slash == input.size() - 1)
        return {};
    return input;
}

// Build source_data JSON for the PowerShell helper.
static std::string BuildSourceDataJson(const ModuleSource &src)
{
    picojson::object o;
    o["id"]          = picojson::value(src.id);
    o["kind"]        = picojson::value(
        src.kind == SourceKind::Folder  ? "folder" :
        src.kind == SourceKind::GitHub  ? "github" : "registry");
    o["label"]       = picojson::value(src.label);
    if (!src.path.empty())       o["path"]       = picojson::value(src.path);
    if (!src.owner_repo.empty()) o["owner_repo"] = picojson::value(src.owner_repo);
    // Registry sources need their base URL passed to face-module-sync.ps1;
    // without this field the script's registry branch aborts at
    // "Registry source has no url field." before issuing any HTTP request,
    // so legacy-registry.whyknot.dev modules never get pulled.
    if (!src.url.empty())        o["url"]        = picojson::value(src.url);
    return picojson::value(o).serialize();
}

static std::string BuildRegistryInstallDataJson(const ModuleSource &src,
                                                const AvailableModule &mod)
{
    picojson::object o;
    o["id"]            = picojson::value(src.id);
    o["kind"]          = picojson::value(std::string("registry"));
    o["label"]         = picojson::value(src.label);
    o["url"]           = picojson::value(src.url.empty() ? mod.registry_url : src.url);
    o["uuid"]          = picojson::value(mod.uuid);
    o["version"]       = picojson::value(mod.version);
    o["name"]          = picojson::value(mod.name);
    o["vendor"]        = picojson::value(mod.vendor);
    if (!mod.payload_url.empty())    o["payload_url"]    = picojson::value(mod.payload_url);
    if (!mod.payload_sha256.empty()) o["payload_sha256"] = picojson::value(mod.payload_sha256);
    if (!mod.download_url.empty())   o["download_url"]   = picojson::value(mod.download_url);
    if (!mod.file_hash.empty())      o["file_hash"]      = picojson::value(mod.file_hash);
    if (!mod.dll_file_name.empty())  o["dll_file_name"]  = picojson::value(mod.dll_file_name);
    return picojson::value(o).serialize();
}

// ---- section helpers ----------------------------------------------------

// Resolve a display label for an installed module's source. Falls back to
// the kind tag from source.json when the catalogue lookup misses (e.g. a
// source that was removed since install, or a source.json that was written
// before the catalogue learned about its id). Distinguishes "no source
// metadata at all" from "metadata present but unmatched in the catalogue".
static std::string ResolveSourceLabel(const InstalledModule &m,
                                      const SourcesCatalogue &cat)
{
    for (const auto &src : cat.sources)
        if (src.id == m.source_id && !src.label.empty()) return src.label;

    if (!m.source_kind_str.empty()) {
        if (m.source_kind_str == "registry") return "Registry";
        if (m.source_kind_str == "github")   return m.release_tag.empty()
                                                     ? std::string{"GitHub"}
                                                     : "GitHub @ " + m.release_tag;
        if (m.source_kind_str == "folder")   return "Folder";
        return m.source_kind_str;
    }
    return "(unknown)";
}

// True if `uuid` is currently in the user's enabled set.
static bool IsModuleEnabled(const FacetrackingPlugin &plugin,
                            const std::string &uuid)
{
    const auto &v = plugin.Profile().current.enabled_module_uuids;
    for (const auto &id : v) if (id == uuid) return true;
    return false;
}

static void DisableModulesFromSource(FacetrackingPlugin &plugin,
                                     const std::string &sourceId)
{
    if (sourceId.empty()) return;

    std::vector<std::string> next = plugin.Profile().current.enabled_module_uuids;
    const size_t before = next.size();
    for (const auto &m : g_state.installed) {
        if (m.source_id != sourceId) continue;
        next.erase(std::remove(next.begin(), next.end(), m.uuid), next.end());
    }

    if (next.size() != before) {
        FT_LOG_OVL("[modules] disabling %zu module(s) before removing source '%s'",
            before - next.size(), sourceId.c_str());
        plugin.SendEnabledModules(next);
    }
}

static const ModuleSource *FindSourceById(const std::string &sourceId)
{
    for (const auto &src : g_state.catalogue.sources)
        if (src.id == sourceId) return &src;
    return nullptr;
}

static const InstalledModule *FindInstalledByUuid(const std::string &uuid)
{
    for (const auto &mod : g_state.installed)
        if (mod.uuid == uuid) return &mod;
    return nullptr;
}

static void DrawInstalledModulesSection(FacetrackingPlugin &plugin)
{
    DrawSectionHeading("Modules");

    if (g_state.installed.empty()) {
        DrawWaitingBanner("No modules installed. Sync a registry source, then install a module from the list.");
        return;
    }

    // Host-confirmed currently-running module UUID, if any.
    const auto &hs = plugin.HostStatus().Snapshot();
    const std::string *hostActiveUuid =
        hs.valid && hs.active_module.has_value() ? &hs.active_module->uuid : nullptr;

    ImGuiTableFlags tf = ImGuiTableFlags_Borders
                       | ImGuiTableFlags_RowBg
                       | ImGuiTableFlags_Resizable
                       | ImGuiTableFlags_SizingStretchProp;

    if (!ImGui::BeginTable("ft_installed_v3", 5, tf)) return;
    ImGui::TableSetupColumn("On",       ImGuiTableColumnFlags_WidthFixed,  36.0f);
    ImGui::TableSetupColumn("Name");
    ImGui::TableSetupColumn("Version",  ImGuiTableColumnFlags_WidthFixed,  80.0f);
    ImGui::TableSetupColumn("Vendor",   ImGuiTableColumnFlags_WidthFixed, 140.0f);
    ImGui::TableSetupColumn("Source",   ImGuiTableColumnFlags_WidthFixed, 180.0f);
    ImGui::TableHeadersRow();

    // Collect the new enabled set, applied once at the end of the table so
    // we don't fire one SendEnabledModules per row when the user clicks a
    // single box. Order: start from the profile so the user's current order
    // is preserved; flip entries whose row got toggled.
    std::vector<std::string> nextEnabled = plugin.Profile().current.enabled_module_uuids;
    bool changed = false;

    for (size_t i = 0; i < g_state.installed.size(); ++i) {
        const auto &m = g_state.installed[i];
        bool enabled = IsModuleEnabled(plugin, m.uuid);

        ImGui::TableNextRow();

        // Column 0: Enabled checkbox.
        ImGui::TableSetColumnIndex(0);
        const std::string boxId = "##ft_en_" + m.uuid;
        if (ImGui::Checkbox(boxId.c_str(), &enabled)) {
            if (enabled) {
                // Append to the end -- preserves the priority order the
                // user has already established.
                bool present = false;
                for (const auto &id : nextEnabled) if (id == m.uuid) { present = true; break; }
                if (!present) nextEnabled.push_back(m.uuid);
            } else {
                nextEnabled.erase(
                    std::remove(nextEnabled.begin(), nextEnabled.end(), m.uuid),
                    nextEnabled.end());
            }
            changed = true;
        }
        TooltipForLastItem("Toggle whether this module is enabled for the host.\n"
                           "Multiple can be on at once; the host currently runs the\n"
                           "first enabled entry (multi-run support is on the roadmap).");

        // Column 1: Name. Tint green when the host confirms this row is the
        // currently running module.
        ImGui::TableSetColumnIndex(1);
        const auto &pal = GetPalette();
        const bool isHostActive = hostActiveUuid && *hostActiveUuid == m.uuid;
        if (isHostActive)
            ImGui::TextColored(pal.statusOk, "%s", m.name.c_str());
        else
            ImGui::TextUnformatted(m.name.c_str());
        if (isHostActive)
            TooltipForLastItem("The host is currently running this module.");

        // Column 2: Version.
        ImGui::TableSetColumnIndex(2);
        ImGui::TextUnformatted(m.version.c_str());

        // Column 3: Vendor.
        ImGui::TableSetColumnIndex(3);
        ImGui::TextUnformatted(m.vendor.c_str());

        // Column 4: Source. Resolves through the catalogue, falls back to
        // the kind tag, and lights up red for GitHub modules whose SHA-256
        // wasn't verified at install time + amber for local folders.
        ImGui::TableSetColumnIndex(4);
        const std::string srcLabel = ResolveSourceLabel(m, g_state.catalogue);
        if (m.source_kind_str == "github" && !m.sha_verified) {
            ImGui::TextColored(pal.statusError,
                               "%s (unverified)", srcLabel.c_str());
            TooltipForLastItem("This module was installed without SHA-256 verification.\n"
                               "The release notes did not contain a recognisable SHA-256 hash.\n"
                               "Confirm the developer publishes verifiable hashes before trusting this source.");
        } else if (m.source_kind_str == "folder") {
            ImGui::TextColored(pal.statusWarn,
                               "%s", srcLabel.c_str());
            TooltipForLastItem("This module was installed from a local folder.\n"
                               "Local modules are not signature-verified.");
        } else {
            ImGui::TextDisabled("%s", srcLabel.c_str());
        }
    }
    ImGui::EndTable();

    if (changed) {
        FT_LOG_OVL("[modules] user changed enabled set: count=%zu",
                   nextEnabled.size());
        plugin.SendEnabledModules(nextEnabled);
    }
}

static void DrawAvailableModulesSection(FacetrackingPlugin &plugin)
{
    if (g_state.available.empty()) return;

    DrawSectionHeading("Available modules");

    ImGuiTableFlags tf = ImGuiTableFlags_Borders
                       | ImGuiTableFlags_RowBg
                       | ImGuiTableFlags_Resizable
                       | ImGuiTableFlags_SizingStretchProp;

    if (!ImGui::BeginTable("ft_available_modules", 5, tf)) return;
    ImGui::TableSetupColumn("Name");
    ImGui::TableSetupColumn("Version", ImGuiTableColumnFlags_WidthFixed, 80.0f);
    ImGui::TableSetupColumn("Vendor",  ImGuiTableColumnFlags_WidthFixed, 140.0f);
    ImGui::TableSetupColumn("Source",  ImGuiTableColumnFlags_WidthFixed, 160.0f);
    ImGui::TableSetupColumn("Action",  ImGuiTableColumnFlags_WidthFixed, 100.0f);
    ImGui::TableHeadersRow();

    std::string installSourceId;
    std::string installSourceData;

    for (const auto &mod : g_state.available) {
        const ModuleSource *source = FindSourceById(mod.source_id);
        const InstalledModule *installed = FindInstalledByUuid(mod.uuid);
        const bool sameVersionInstalled =
            installed && installed->version == mod.version;
        const bool sourceMissing = source == nullptr;

        ImGui::TableNextRow();

        ImGui::TableSetColumnIndex(0);
        ImGui::TextUnformatted(mod.name.c_str());
        if (!mod.description.empty())
            TooltipForLastItem(mod.description.c_str());

        ImGui::TableSetColumnIndex(1);
        ImGui::TextUnformatted(mod.version.c_str());

        ImGui::TableSetColumnIndex(2);
        ImGui::TextUnformatted(mod.vendor.c_str());

        ImGui::TableSetColumnIndex(3);
        ImGui::TextDisabled("%s", mod.source_label.c_str());

        ImGui::TableSetColumnIndex(4);
        const bool busy = plugin.SyncRunner().IsRunning();
        const bool disabled = busy || sameVersionInstalled || sourceMissing;
        const char *label =
            sameVersionInstalled ? "Installed" :
            installed            ? "Update" :
                                   "Install";
        ImGui::BeginDisabled(disabled);
        if (ImGui::SmallButton((std::string(label) + "##install_" + mod.uuid).c_str())) {
            installSourceId = mod.source_id;
            installSourceData = BuildRegistryInstallDataJson(*source, mod);
        }
        ImGui::EndDisabled();
        if (sameVersionInstalled) {
            TooltipForLastItem("This version is already installed.");
        } else if (sourceMissing) {
            TooltipForLastItem("The source for this cached module is no longer configured.");
        } else {
            TooltipForLastItem("Download and install this module.");
        }
    }
    ImGui::EndTable();

    if (!installSourceId.empty() && !plugin.SyncRunner().IsRunning())
        plugin.SyncRunner().StartInstall(installSourceId, installSourceData);
}

static void DrawSourcesSection(FacetrackingPlugin &plugin)
{
    DrawSectionHeading("Module sources");

    while (auto result = plugin.ConsumeSyncResult()) {
        g_state.sync_status    = result->message;
        g_state.sync_status_ok = result->ok;
        g_state.last_scan_tick = 0;
        RefreshIfStale();
    }

    // ---- status line ----
    if (!g_state.sync_status.empty()) {
        const auto &palSync = GetPalette();
        if (g_state.sync_status_ok)
            ImGui::TextColored(palSync.statusOk,
                               "Sync: %s", g_state.sync_status.c_str());
        else
            ImGui::TextColored(palSync.statusError,
                               "Sync error: %s", g_state.sync_status.c_str());
    } else if (plugin.SyncRunner().IsRunning()) {
        ImGui::TextDisabled("Syncing...");
    }

    // ---- sources table ----
    ImGuiTableFlags tf = ImGuiTableFlags_Borders
                       | ImGuiTableFlags_RowBg
                       | ImGuiTableFlags_SizingStretchProp;

    if (ImGui::BeginTable("ft_sources", 4, tf)) {
        ImGui::TableSetupColumn("Label");
        ImGui::TableSetupColumn("Kind",      ImGuiTableColumnFlags_WidthFixed,  70.0f);
        ImGui::TableSetupColumn("Status",    ImGuiTableColumnFlags_WidthFixed, 140.0f);
        ImGui::TableSetupColumn("Actions",   ImGuiTableColumnFlags_WidthFixed, 180.0f);
        ImGui::TableHeadersRow();

        int  removeIdx = -1;
        std::string syncSourceId;
        std::string syncSourceData;

        for (int si = 0; si < static_cast<int>(g_state.catalogue.sources.size()); ++si) {
            auto &src = g_state.catalogue.sources[static_cast<size_t>(si)];
            bool isRegistry = src.kind == SourceKind::Registry;

            ImGui::TableNextRow();
            ImGui::TableSetColumnIndex(0);
            ImGui::TextUnformatted(src.label.c_str());

            ImGui::TableSetColumnIndex(1);
            const char *kindStr =
                src.kind == SourceKind::Folder  ? "Folder"   :
                src.kind == SourceKind::GitHub  ? "GitHub"   : "Registry";
            ImGui::TextDisabled("%s", kindStr);

            ImGui::TableSetColumnIndex(2);
            if (!src.last_sync_error.empty())
                ImGui::TextColored(GetPalette().statusError,
                                   "%s", src.last_sync_error.c_str());
            else if (!src.last_checked_at.empty())
                ImGui::TextDisabled("%s", src.last_checked_at.c_str());
            else
                ImGui::TextDisabled("Never synced");

            ImGui::TableSetColumnIndex(3);
            bool syncing = plugin.SyncRunner().IsRunning();

            ImGui::BeginDisabled(syncing);
            const char *buttonText = isRegistry ? "Sync" : "Install";
            if (ImGui::SmallButton((std::string(buttonText) + "##" + src.id).c_str())) {
                syncSourceId   = src.id;
                syncSourceData = BuildSourceDataJson(src);
            }
            TooltipForLastItem(isRegistry
                ? "Refresh the available module list. Downloads start only from a module row below."
                : "Install or re-install this source.");

            if (!isRegistry) {
                ImGui::SameLine();
                if (ImGui::SmallButton(("Remove##" + src.id).c_str()))
                    removeIdx = si;
                TooltipForLastItem("Remove this source and delete its installed modules.");
            }
            ImGui::EndDisabled();
        }

        // Trigger sync outside the table loop to avoid iterator invalidation.
        if (!syncSourceId.empty() && !plugin.SyncRunner().IsRunning())
            plugin.SyncRunner().StartUpdate(syncSourceId, syncSourceData);

        if (removeIdx >= 0) {
            const auto &src = g_state.catalogue.sources[static_cast<size_t>(removeIdx)];
            std::string id = src.id;
            DisableModulesFromSource(plugin, id);
            plugin.SyncRunner().StartRemove(id);
            g_state.catalogue.sources.erase(
                g_state.catalogue.sources.begin() + removeIdx);
            SaveSourcesCatalogue(g_state.catalogue);
        }

        ImGui::EndTable();
    }

    DrawAvailableModulesSection(plugin);

    // ---- add buttons ----
    ImGui::Spacing();
    bool syncing = plugin.SyncRunner().IsRunning();

    ImGui::BeginDisabled(syncing);
    if (ImGui::Button("Add folder source...##ft_add_folder")) {
        std::string chosen = PickFolder();
        if (!chosen.empty()) {
            ModuleSource src;
            src.id       = GenerateSourceId();
            src.kind     = SourceKind::Folder;
            src.path     = chosen;
            // Label = last path component.
            size_t slash = chosen.find_last_of("\\/");
            src.label    = "Folder: " + (slash != std::string::npos
                               ? chosen.substr(slash + 1) : chosen);
            src.added_at = NowIso8601();
            std::string data = BuildSourceDataJson(src);
            g_state.catalogue.sources.push_back(src);
            SaveSourcesCatalogue(g_state.catalogue);
            plugin.SyncRunner().StartAdd(SourceKind::Folder, data, src.id);
        }
    }
    TooltipForLastItem("Pick a local folder containing a packaged face-tracking module.");

    ImGui::SameLine();
    if (ImGui::Button("Add GitHub repo...##ft_add_github"))
        ImGui::OpenPopup("ft_add_github_popup");
    TooltipForLastItem("Fetch the latest release from a GitHub repository.");
    ImGui::EndDisabled();

    if (ImGui::BeginPopupModal("ft_add_github_popup", nullptr,
                               ImGuiWindowFlags_AlwaysAutoResize)) {
        ImGui::Text("GitHub repository URL or owner/repo:");
        ImGui::SetNextItemWidth(360.0f);
        ImGui::InputText("##ft_github_url", g_state.github_input,
                         sizeof(g_state.github_input));
        TooltipForLastItem("e.g. https://github.com/owner/repo  or  owner/repo");
        ImGui::Spacing();
        if (ImGui::Button("Add##ft_github_add", ImVec2(110, 0))) {
            std::string ownerRepo = ParseOwnerRepo(std::string(g_state.github_input));
            if (!ownerRepo.empty()) {
                ModuleSource src;
                src.id          = GenerateSourceId();
                src.kind        = SourceKind::GitHub;
                src.owner_repo  = ownerRepo;
                src.label       = ownerRepo;
                src.auto_update = false;
                src.added_at    = NowIso8601();
                std::string data = BuildSourceDataJson(src);
                g_state.catalogue.sources.push_back(src);
                SaveSourcesCatalogue(g_state.catalogue);
                plugin.SyncRunner().StartAdd(SourceKind::GitHub, data, src.id);
                g_state.github_input[0] = '\0';
                ImGui::CloseCurrentPopup();
            }
            // If ownerRepo empty: leave popup open, no-op (bad format).
        }
        ImGui::SameLine();
        if (ImGui::Button("Cancel##ft_github_cancel", ImVec2(110, 0)))
            ImGui::CloseCurrentPopup();
        ImGui::EndPopup();
    }

}

static void DrawTrustBanners()
{
    bool hasNonRegistry = false;
    bool hasUnverifiedGithub = false;
    for (const auto &src : g_state.catalogue.sources) {
        if (src.kind != SourceKind::Registry) hasNonRegistry = true;
    }
    for (const auto &m : g_state.installed) {
        if (m.source_kind_str == "github" && !m.sha_verified) {
            hasUnverifiedGithub = true;
            break;
        }
    }

    if (hasNonRegistry) {
        DrawBanner(
            "Untrusted sources",
            "Folder and GitHub sources are untrusted. They can execute arbitrary "
            "code in the face-tracking host process. Only add sources from "
            "developers you trust.",
            ImVec4(0.38f, 0.30f, 0.02f, 1.0f),
            ImVec4(1.0f, 0.92f, 0.45f, 1.0f),
            ImVec4(1.0f, 0.96f, 0.80f, 1.0f));
    }

    if (hasUnverifiedGithub) {
        DrawBanner(
            "Unverified SHA-256",
            "One or more GitHub modules were installed without SHA-256 verification "
            "because the release notes did not contain a SHA-256 hash. Confirm the "
            "developer publishes verifiable hashes before continuing to trust this source.",
            ImVec4(0.42f, 0.06f, 0.06f, 1.0f),
            ImVec4(1.0f, 0.88f, 0.88f, 1.0f),
            ImVec4(1.0f, 0.96f, 0.96f, 1.0f));
    }
}

// ---- main entry point ---------------------------------------------------

void DrawModulesTab(FacetrackingPlugin &plugin)
{
    RefreshIfStale();

    DrawInstalledModulesSection(plugin);
    DrawSourcesSection(plugin);
    DrawTrustBanners();
}

} // namespace facetracking::ui
