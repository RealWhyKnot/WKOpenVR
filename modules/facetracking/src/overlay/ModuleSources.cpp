#define _CRT_SECURE_NO_DEPRECATE
#include "ModuleSources.h"

#include "JsonUtil.h"
#include "Logging.h"
#include "Win32Paths.h"
#include "Win32Text.h"

#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#include <bcrypt.h>
#include <wincrypt.h>

#pragma comment(lib, "bcrypt.lib")

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <ctime>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

namespace facetracking {

namespace fs = std::filesystem;

// ---- public utilities ---------------------------------------------------

std::wstring FtDataDir()
{
    return openvr_pair::common::WkOpenVrSubdirectoryPath(L"facetracking", true);
}

std::string GenerateSourceId()
{
    BYTE buf[16] = {};
    BCryptGenRandom(nullptr, buf, sizeof(buf), BCRYPT_USE_SYSTEM_PREFERRED_RNG);
    char hex[33] = {};
    for (int i = 0; i < 16; ++i)
        (void)snprintf(hex + i * 2, 3, "%02x", buf[i]);
    return std::string(hex);
}

std::string NowIso8601()
{
    SYSTEMTIME st{};
    GetSystemTime(&st);
    char buf[32];
    (void)snprintf(buf, sizeof(buf), "%04d-%02d-%02dT%02d:%02d:%02dZ",
        st.wYear, st.wMonth, st.wDay,
        st.wHour, st.wMinute, st.wSecond);
    return std::string(buf);
}

// ---- sources.json -------------------------------------------------------

static std::wstring SourcesPath()
{
    return FtDataDir() + L"\\sources.json";
}

static SourceKind KindFromString(const std::string &s)
{
    if (s == "folder")   return SourceKind::Folder;
    if (s == "github")   return SourceKind::GitHub;
    return SourceKind::Registry;
}

static std::string KindToString(SourceKind k)
{
    switch (k) {
        case SourceKind::Folder:   return "folder";
        case SourceKind::GitHub:   return "github";
        default:                   return "registry";
    }
}

SourcesCatalogue LoadSourcesCatalogue()
{
    std::wstring path = SourcesPath();
    std::ifstream in(path);
    if (!in.is_open()) return {};

    std::stringstream ss;
    ss << in.rdbuf();
    std::string txt = ss.str();
    picojson::value root;
    if (!openvr_pair::common::json::ParseObject(root, txt)) return {};

    SourcesCatalogue cat;
    cat.schema_version = openvr_pair::common::json::IntAt(root, "schema_version", cat.schema_version);

    if (const auto *arr = openvr_pair::common::json::ArrayAt(root, "sources")) {
        for (const auto &el : *arr) {
            ModuleSource src;
            src.id               = openvr_pair::common::json::StringAt(el, "id");
            src.kind             = KindFromString(openvr_pair::common::json::StringAt(el, "kind"));
            src.url              = openvr_pair::common::json::StringAt(el, "url");
            src.path             = openvr_pair::common::json::StringAt(el, "path");
            src.owner_repo       = openvr_pair::common::json::StringAt(el, "owner_repo");
            src.label            = openvr_pair::common::json::StringAt(el, "label");
            src.auto_update      = openvr_pair::common::json::BoolAt(el, "auto_update");
            src.added_at         = openvr_pair::common::json::StringAt(el, "added_at");
            src.last_checked_at  = openvr_pair::common::json::StringAt(el, "last_checked_at");
            src.last_release_tag = openvr_pair::common::json::StringAt(el, "last_release_tag");
            src.last_sync_error  = openvr_pair::common::json::StringAt(el, "last_sync_error");
            if (!src.id.empty())
                cat.sources.push_back(std::move(src));
        }
    }
    return cat;
}

bool SaveSourcesCatalogue(const SourcesCatalogue &cat)
{
    std::wstring path = SourcesPath();
    std::wstring tmp  = path + L".tmp";

    picojson::array arr;
    for (const auto &src : cat.sources) {
        picojson::object o;
        o["id"]               = picojson::value(src.id);
        o["kind"]             = picojson::value(KindToString(src.kind));
        o["label"]            = picojson::value(src.label);
        o["auto_update"]      = picojson::value(src.auto_update);
        if (!src.url.empty())          o["url"]              = picojson::value(src.url);
        if (!src.path.empty())         o["path"]             = picojson::value(src.path);
        if (!src.owner_repo.empty())   o["owner_repo"]       = picojson::value(src.owner_repo);
        if (!src.added_at.empty())     o["added_at"]         = picojson::value(src.added_at);
        if (!src.last_checked_at.empty())
            o["last_checked_at"] = picojson::value(src.last_checked_at);
        if (!src.last_release_tag.empty())
            o["last_release_tag"] = picojson::value(src.last_release_tag);
        if (!src.last_sync_error.empty())
            o["last_sync_error"] = picojson::value(src.last_sync_error);
        arr.push_back(picojson::value(o));
    }
    picojson::object root;
    root["schema_version"] = picojson::value((double)cat.schema_version);
    root["sources"]        = picojson::value(arr);
    std::string body = picojson::value(root).serialize(true);

    HANDLE hf = CreateFileW(tmp.c_str(), GENERIC_WRITE, 0, nullptr,
                             CREATE_ALWAYS, FILE_ATTRIBUTE_NORMAL, nullptr);
    if (hf == INVALID_HANDLE_VALUE) {
        FT_LOG_OVL("[sources] failed to open sources.json.tmp for write (err=%lu)",
                   GetLastError());
        return false;
    }
    DWORD written = 0;
    BOOL ok = WriteFile(hf, body.data(), (DWORD)body.size(), &written, nullptr);
    if (ok) ok = FlushFileBuffers(hf);
    CloseHandle(hf);

    if (!ok || written != (DWORD)body.size()) {
        FT_LOG_OVL("[sources] write failed for sources.json.tmp (err=%lu)", GetLastError());
        DeleteFileW(tmp.c_str());
        return false;
    }
    if (!MoveFileExW(tmp.c_str(), path.c_str(),
                     MOVEFILE_REPLACE_EXISTING | MOVEFILE_WRITE_THROUGH)) {
        FT_LOG_OVL("[sources] atomic rename failed (err=%lu)", GetLastError());
        DeleteFileW(tmp.c_str());
        return false;
    }
    FT_LOG_OVL("[sources] saved sources.json (%zu sources)", cat.sources.size());
    return true;
}

// Built-in registry source ID (stable across installs).
static const char kRegistrySourceId[] = "00000000000000000000000000000001";
static const char kRegistryUrl[]      = "https://registry.vrcft.io";

SourcesCatalogue EnsureSourcesCatalogue()
{
    SourcesCatalogue cat = LoadSourcesCatalogue();

    // Check if registry entry already exists.
    for (const auto &s : cat.sources)
        if (s.id == kRegistrySourceId) return cat;

    // Seed with registry.
    ModuleSource reg;
    reg.id          = kRegistrySourceId;
    reg.kind        = SourceKind::Registry;
    reg.url         = kRegistryUrl;
    reg.label       = "VRCFT registry";
    reg.auto_update = false;
    cat.sources.insert(cat.sources.begin(), std::move(reg));
    cat.schema_version = 1;
    SaveSourcesCatalogue(cat);
    return cat;
}

std::string SourceLabel(const SourcesCatalogue &cat, const std::string &source_id)
{
    for (const auto &src : cat.sources)
        if (src.id == source_id) return src.label;
    return source_id.empty() ? "Unknown" : source_id;
}

// ---- disk scan ----------------------------------------------------------

std::vector<InstalledModule> ScanInstalledModules()
{
    std::wstring base = FtDataDir();
    if (base.empty()) return {};

    std::wstring modsDir = base + L"\\modules";
    CreateDirectoryW(modsDir.c_str(), nullptr);

    std::vector<InstalledModule> result;

    std::error_code ec;
    for (const auto &uuidEntry : fs::directory_iterator(modsDir, ec)) {
        if (!uuidEntry.is_directory()) continue;
        std::string uuid = openvr_pair::common::WideToUtf8(uuidEntry.path().filename().wstring());

        for (const auto &verEntry : fs::directory_iterator(uuidEntry.path(), ec)) {
            if (!verEntry.is_directory()) continue;
            std::string version = openvr_pair::common::WideToUtf8(verEntry.path().filename().wstring());

            fs::path manifestPath = verEntry.path() / L"manifest.json";
            if (!fs::exists(manifestPath, ec)) continue;

            InstalledModule mod;
            mod.uuid     = uuid;
            mod.version  = version;
            mod.manifest_path = openvr_pair::common::WideToUtf8(manifestPath.wstring());

            // Read manifest.json for name + vendor.
            {
                std::ifstream mf(manifestPath.wstring());
                if (mf.is_open()) {
                    std::stringstream ss;
                    ss << mf.rdbuf();
                    std::string txt = ss.str();
                    picojson::value root;
                    if (openvr_pair::common::json::Parse(root, txt)) {
                        mod.name   = openvr_pair::common::json::StringAt(root, "name");
                        mod.vendor = openvr_pair::common::json::StringAt(root, "vendor");
                    }
                }
            }
            if (mod.name.empty())   mod.name   = uuid;
            if (mod.vendor.empty()) mod.vendor  = "Unknown";

            // Read optional source.json sidecar.
            fs::path sourcePath = verEntry.path() / L"source.json";
            if (fs::exists(sourcePath, ec)) {
                std::ifstream sf(sourcePath.wstring());
                if (sf.is_open()) {
                    std::stringstream ss;
                    ss << sf.rdbuf();
                    std::string txt = ss.str();
                    picojson::value root;
                    if (openvr_pair::common::json::Parse(root, txt)) {
                        mod.source_id       = openvr_pair::common::json::StringAt(root, "source_id");
                        mod.source_kind_str = openvr_pair::common::json::StringAt(root, "source_kind");
                        mod.sha_verified    = openvr_pair::common::json::BoolAt(root, "verified_sha256");
                        mod.release_tag     = openvr_pair::common::json::StringAt(root, "release_tag");
                    }
                }
            }

            result.push_back(std::move(mod));
        }
    }
    std::sort(result.begin(), result.end(), [](const InstalledModule &a, const InstalledModule &b) {
        const int byName = _stricmp(a.name.c_str(), b.name.c_str());
        if (byName != 0) return byName < 0;
        const int byVersion = _stricmp(a.version.c_str(), b.version.c_str());
        if (byVersion != 0) return byVersion < 0;
        return a.uuid < b.uuid;
    });
    return result;
}

std::vector<AvailableModule> LoadAvailableModules(const std::string &source_id)
{
    std::wstring base = FtDataDir();
    if (base.empty()) return {};

    fs::path availableDir = fs::path(base) / L"available";
    std::error_code ec;
    fs::create_directories(availableDir, ec);

    std::vector<fs::path> files;
    if (!source_id.empty()) {
        files.push_back(availableDir / (openvr_pair::common::Utf8ToWide(source_id) + L".json"));
    } else {
        for (const auto &entry : fs::directory_iterator(availableDir, ec)) {
            if (entry.is_regular_file() && entry.path().extension() == L".json")
                files.push_back(entry.path());
        }
    }

    std::vector<AvailableModule> result;
    for (const auto &path : files) {
        if (!fs::exists(path, ec)) continue;

        std::ifstream in(path.wstring());
        if (!in.is_open()) continue;
        std::stringstream ss;
        ss << in.rdbuf();

        picojson::value root;
        if (!openvr_pair::common::json::ParseObject(root, ss.str())) continue;

        const std::string sourceId = openvr_pair::common::json::StringAt(root, "source_id");
        if (!source_id.empty() && sourceId != source_id) continue;
        const std::string sourceLabel = openvr_pair::common::json::StringAt(root, "source_label");
        const std::string registryUrl = openvr_pair::common::json::StringAt(root, "registry_url");

        const auto *arr = openvr_pair::common::json::ArrayAt(root, "modules");
        if (!arr) continue;

        for (const auto &el : *arr) {
            AvailableModule mod;
            mod.uuid           = openvr_pair::common::json::StringAt(el, "uuid");
            mod.version        = openvr_pair::common::json::StringAt(el, "version");
            mod.name           = openvr_pair::common::json::StringAt(el, "name", mod.uuid);
            mod.vendor         = openvr_pair::common::json::StringAt(el, "vendor", "Unknown");
            mod.description    = openvr_pair::common::json::StringAt(el, "description");
            mod.source_id      = openvr_pair::common::json::StringAt(el, "source_id", sourceId);
            mod.source_label   = openvr_pair::common::json::StringAt(el, "source_label", sourceLabel);
            mod.registry_url   = openvr_pair::common::json::StringAt(el, "registry_url", registryUrl);
            mod.payload_url    = openvr_pair::common::json::StringAt(el, "payload_url");
            mod.payload_sha256 = openvr_pair::common::json::StringAt(el, "payload_sha256");
            mod.download_url   = openvr_pair::common::json::StringAt(el, "download_url");
            mod.file_hash      = openvr_pair::common::json::StringAt(el, "file_hash");
            mod.dll_file_name  = openvr_pair::common::json::StringAt(el, "dll_file_name");
            mod.module_page_url = openvr_pair::common::json::StringAt(el, "module_page_url");
            if (mod.uuid.empty()) continue;
            if (mod.name.empty()) mod.name = mod.uuid;
            result.push_back(std::move(mod));
        }
    }

    std::sort(result.begin(), result.end(), [](const AvailableModule &a, const AvailableModule &b) {
        const int byName = _stricmp(a.name.c_str(), b.name.c_str());
        if (byName != 0) return byName < 0;
        return a.uuid < b.uuid;
    });
    return result;
}

// ---- sync runner --------------------------------------------------------

ModuleSyncRunner::ModuleSyncRunner() = default;

ModuleSyncRunner::~ModuleSyncRunner()
{
    if (proc_ != INVALID_HANDLE_VALUE) {
        TerminateProcess(proc_, 1);
        CloseHandle(proc_);
    }
}

/*static*/
std::wstring ModuleSyncRunner::ScriptPath()
{
    wchar_t exePath[MAX_PATH] = {};
    GetModuleFileNameW(nullptr, exePath, MAX_PATH);
    fs::path p(exePath);
    return (p.parent_path() / L"resources" / L"face-module-sync.ps1").wstring();
}

void ModuleSyncRunner::StartAdd(SourceKind kind, const std::string &source_data_json,
                                const std::string &source_id)
{
    if (queue_.size() >= 8) return;
    PendingOp op;
    op.action      = "add";
    op.kind        = KindToString(kind);
    op.source_data = source_data_json;
    op.source_id   = source_id;
    queue_.push_back(std::move(op));
    if (!IsRunning()) LaunchNext();
}

void ModuleSyncRunner::StartUpdate(const std::string &source_id,
                                   const std::string &source_data_json)
{
    if (queue_.size() >= 8) return;
    PendingOp op;
    op.action      = "update";
    op.source_data = source_data_json;
    op.source_id   = source_id;
    queue_.push_back(std::move(op));
    if (!IsRunning()) LaunchNext();
}

void ModuleSyncRunner::StartInstall(const std::string &source_id,
                                    const std::string &source_data_json)
{
    if (queue_.size() >= 8) return;
    PendingOp op;
    op.action      = "install";
    op.kind        = "registry";
    op.source_data = source_data_json;
    op.source_id   = source_id;
    queue_.push_back(std::move(op));
    if (!IsRunning()) LaunchNext();
}

void ModuleSyncRunner::StartRemove(const std::string &source_id)
{
    if (queue_.size() >= 8) return;
    PendingOp op;
    op.action    = "remove";
    op.source_id = source_id;
    queue_.push_back(std::move(op));
    if (!IsRunning()) LaunchNext();
}

bool ModuleSyncRunner::IsRunning() const
{
    return proc_ != INVALID_HANDLE_VALUE;
}

std::optional<SyncResult> ModuleSyncRunner::Poll()
{
    if (immediate_result_.has_value()) {
        SyncResult r = *immediate_result_;
        immediate_result_.reset();
        if (!IsRunning() && !queue_.empty()) LaunchNext();
        return r;
    }

    if (!IsRunning()) {
        if (!queue_.empty()) LaunchNext();
        return std::nullopt;
    }

    // Check if the script process finished.
    DWORD exitCode = STILL_ACTIVE;
    if (!GetExitCodeProcess(proc_, &exitCode) || exitCode == STILL_ACTIVE) {
        // Still running -- check for timeout (90 s).
        auto elapsed = std::chrono::steady_clock::now() - launch_time_;
        if (elapsed > std::chrono::seconds(90)) {
            FT_LOG_OVL("[sync] script timed out after 90 s, killing");
            TerminateProcess(proc_, 1);
            CloseHandle(proc_);
            proc_ = INVALID_HANDLE_VALUE;
            SyncResult r;
            r.ok      = false;
            r.message = "Sync timed out after 90 s.";
            r.source_id = active_op_.source_id;
            r.action    = active_op_.action;
            if (!queue_.empty()) LaunchNext();
            return r;
        }
        return std::nullopt;
    }

    CloseHandle(proc_);
    proc_ = INVALID_HANDLE_VALUE;

    // Read the result JSON.
    SyncResult result;
    if (!result_path_.empty()) {
        std::ifstream rf(result_path_);
        if (rf.is_open()) {
            std::stringstream ss;
            ss << rf.rdbuf();
            std::string txt = ss.str();
            picojson::value root;
            if (openvr_pair::common::json::Parse(root, txt)) {
                result.ok                = openvr_pair::common::json::BoolAt(root, "ok");
                result.message           = openvr_pair::common::json::StringAt(root, "message");
                result.installed_uuid    = openvr_pair::common::json::StringAt(root, "installed_uuid");
                result.installed_version = openvr_pair::common::json::StringAt(root, "installed_version");
                result.source_id         = openvr_pair::common::json::StringAt(root, "source_id", active_op_.source_id);
                result.action            = openvr_pair::common::json::StringAt(root, "action", active_op_.action);
                result.available_count   = openvr_pair::common::json::IntAt(root, "available_count", -1);
            } else {
                result.ok      = false;
                result.message = "Result JSON parse error.";
                result.source_id = active_op_.source_id;
                result.action    = active_op_.action;
            }
            rf.close();
            DeleteFileW(result_path_.c_str());
        } else {
            result.ok      = exitCode == 0;
            result.message = exitCode == 0 ? "Done." : "Sync failed -- check log.";
            result.source_id = active_op_.source_id;
            result.action    = active_op_.action;
        }
        result_path_.clear();
    }

    FT_LOG_OVL("[sync] script finished: ok=%d msg='%s'",
               (int)result.ok, result.message.c_str());

    if (!queue_.empty()) LaunchNext();
    return result;
}

// Encode a UTF-8 string as the base64 UTF-16 LE payload for
// powershell.exe -EncodedCommand.
static std::string EncodeForPowerShell(const std::wstring &cmd)
{
    // Base64-encode the UTF-16 LE byte sequence.
    const BYTE *bytes = reinterpret_cast<const BYTE *>(cmd.data());
    DWORD blen = static_cast<DWORD>(cmd.size() * sizeof(wchar_t));
    DWORD outlen = 0;
    CryptBinaryToStringA(bytes, blen, CRYPT_STRING_BASE64 | CRYPT_STRING_NOCRLF,
                         nullptr, &outlen);
    std::string out(outlen, '\0');
    CryptBinaryToStringA(bytes, blen, CRYPT_STRING_BASE64 | CRYPT_STRING_NOCRLF,
                         out.data(), &outlen);
    // outlen includes null terminator if any; trim trailing null.
    while (!out.empty() && out.back() == '\0') out.pop_back();
    return out;
}

#pragma comment(lib, "crypt32.lib")

void ModuleSyncRunner::LaunchNext()
{
    if (queue_.empty()) return;
    PendingOp op = queue_.front();
    queue_.erase(queue_.begin());
    active_op_ = op;

    // Build a temp result path.
    wchar_t tmp[MAX_PATH], dir[MAX_PATH];
    GetTempPathW(MAX_PATH, dir);
    GetTempFileNameW(dir, L"fts", 0, tmp);
    result_path_ = std::wstring(tmp);

    std::wstring script = ScriptPath();

    // Wrap argument values in PowerShell single-quoted strings. These are
    // fully literal -- the only escape is '' for an embedded apostrophe.
    // Backslashes, double quotes, braces, dollar signs, etc. all pass through
    // untouched. This is the only quoting form that's safe for arbitrary
    // JSON content. The earlier escapeJs used POSIX/CMD rules (\\\\, \\")
    // which PowerShell does not honour -- PS would treat \" inside a
    // double-quoted argument as a literal \ followed by a string-terminator,
    // ending the SourceData arg prematurely and causing the rest of the JSON
    // to be parsed as command tokens (observed 2026-05-13: PS errored on
    // "curated" -- the word after the broken-out parenthesis in a label --
    // and the script never ran, so the result file stayed empty and picojson
    // reported "Result JSON parse error").
    auto psSingleQuoteWide = [](const std::wstring& wide) -> std::wstring {
        std::wstring r;
        r.reserve(wide.size() + 2);
        r += L'\'';
        for (wchar_t c : wide) {
            if (c == L'\'') r += L"''";
            else            r += c;
        }
        r += L'\'';
        return r;
    };
    auto psSingleQuote = [&](const std::string& utf8) -> std::wstring {
        return psSingleQuoteWide(openvr_pair::common::Utf8ToWide(utf8));
    };

    // Build the PS command as a wstring, then base64 encode it.
    std::wstring psCmd = L"& '" + script + L"'";
    psCmd += L" -Action " + psSingleQuote(op.action);
    if (!op.kind.empty())
        psCmd += L" -Kind " + psSingleQuote(op.kind);
    if (!op.source_data.empty())
        psCmd += L" -SourceData " + psSingleQuote(op.source_data);
    if (!op.source_id.empty())
        psCmd += L" -SourceId " + psSingleQuote(op.source_id);
    psCmd += L" -ResultPath " + psSingleQuoteWide(result_path_);

    std::string encoded = EncodeForPowerShell(psCmd);

    std::wstring cmdLine = L"powershell.exe -NoProfile -NonInteractive -ExecutionPolicy Bypass -EncodedCommand ";
    cmdLine += openvr_pair::common::Utf8ToWide(encoded);

    STARTUPINFOW si{};
    si.cb          = sizeof(si);
    si.dwFlags     = STARTF_USESHOWWINDOW;
    si.wShowWindow = SW_HIDE;
    PROCESS_INFORMATION pi{};

    std::vector<wchar_t> cmdBuf(cmdLine.begin(), cmdLine.end());
    cmdBuf.push_back(L'\0');

    if (!CreateProcessW(nullptr, cmdBuf.data(), nullptr, nullptr, FALSE,
                        CREATE_NO_WINDOW, nullptr, nullptr, &si, &pi)) {
        FT_LOG_OVL("[sync] CreateProcessW failed: err=%lu", GetLastError());
        result_path_.clear();
        SyncResult r;
        r.ok      = false;
        r.message = "Failed to launch sync script.";
        r.source_id = op.source_id;
        r.action    = op.action;
        immediate_result_ = std::move(r);
        return;
    }

    CloseHandle(pi.hThread);
    proc_        = pi.hProcess;
    launch_time_ = std::chrono::steady_clock::now();
    FT_LOG_OVL("[sync] launched script: action=%s kind=%s",
               op.action.c_str(), op.kind.c_str());
}

} // namespace facetracking
