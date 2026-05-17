#include "DiscordPresenceComposer.h"
#include "DiscordPresence.h"

#include "DebugLogging.h"

#include <chrono>
#include <cstring>
#include <string>

namespace WKOpenVR {

namespace {

// Idle carousel: when no module has priority > 0, cycle through submissions
// at this interval so the user sees each module in turn.
constexpr double kIdleCarouselSec = 6.0;

// Discord documented limits (bytes, not chars). state/details/largeImageText
// are 128; smallImageKey / largeImageKey are 32. We only use the text fields.
constexpr size_t kDiscordMaxTextBytes = 128;

// Wall-clock seconds since epoch, via steady_clock for monotonic deltas.
using Clock = std::chrono::steady_clock;
double NowSec()
{
    return std::chrono::duration<double>(Clock::now().time_since_epoch()).count();
}

// Clamp a UTF-8 string to at most maxBytes bytes without splitting a
// multi-byte codepoint. Returns the truncated string.
std::string ClampUtf8(std::string s, size_t maxBytes)
{
    if (s.size() <= maxBytes) return s;

    // Walk back from the maxBytes boundary until we land on a byte that is
    // not a UTF-8 continuation byte (0x80..0xBF). That gives us the start of
    // the last partial codepoint, which we then exclude.
    size_t cut = maxBytes;
    while (cut > 0 && (static_cast<unsigned char>(s[cut]) & 0xC0) == 0x80) {
        --cut;
    }
    s.resize(cut);
    return s;
}

} // namespace

void PresenceComposer::BeginFrame()
{
    entries_.clear();
}

void PresenceComposer::Submit(const std::string &moduleName, PresenceUpdate update)
{
    update.details        = ClampUtf8(std::move(update.details),        kDiscordMaxTextBytes);
    update.state          = ClampUtf8(std::move(update.state),          kDiscordMaxTextBytes);
    update.largeImageText = ClampUtf8(std::move(update.largeImageText), kDiscordMaxTextBytes);

    entries_.push_back({moduleName, std::move(update)});
}

void PresenceComposer::Tick()
{
    if (entries_.empty()) {
        // Nothing submitted -- idle fallback.
        const std::string details = "WKOpenVR ready";
        const std::string state   = "No modules enabled";
        const std::string imgText = "WKOpenVR";
        if (details != lastDetails_ || state != lastState_ || imgText != lastLargeImageText_) {
            if (openvr_pair::common::IsDebugLoggingEnabled()) {
                DiscordPresence_LogInfo(
                    "[composer] no-submission idle fallback -> details='%s' state='%s'",
                    details.c_str(), state.c_str());
            }
            DiscordPresence_SetState(state.c_str(), details.c_str());
            lastDetails_        = details;
            lastState_          = state;
            lastLargeImageText_ = imgText;
            lastWinnerModule_   = "";
            lastWinnerPriority_ = -1;
        }
        return;
    }

    // Find the highest priority among all submissions.
    int maxPriority = 0;
    for (const auto &e : entries_) {
        if (e.update.priority > maxPriority) maxPriority = e.update.priority;
    }

    const double now = NowSec();

    size_t winnerIdx = 0;
    if (maxPriority == 0) {
        // Idle carousel: advance once every kIdleCarouselSec seconds.
        if (now - lastCarouselSec_ >= kIdleCarouselSec) {
            lastCarouselSec_ = now;
            ++carouselIndex_;
        }
        winnerIdx = carouselIndex_ % entries_.size();
    } else {
        // Non-idle: pick the first entry at the highest priority (stable order).
        for (size_t i = 0; i < entries_.size(); ++i) {
            if (entries_[i].update.priority == maxPriority) {
                winnerIdx = i;
                break;
            }
        }
        // Reset carousel so it starts fresh next time we return to idle.
        carouselIndex_  = 0;
        lastCarouselSec_ = now;
    }

    const Entry &winner = entries_[winnerIdx];
    const std::string &details  = winner.update.details;
    const std::string &state    = winner.update.state;
    std::string imgText = winner.update.largeImageText.empty()
        ? "WKOpenVR"
        : winner.update.largeImageText;
    imgText = ClampUtf8(imgText, kDiscordMaxTextBytes);

    const bool winnerChanged =
        winner.moduleName != lastWinnerModule_ ||
        winner.update.priority != lastWinnerPriority_ ||
        details != lastDetails_ ||
        state   != lastState_   ||
        imgText != lastLargeImageText_;

    if (winnerChanged) {
        // Debug-mode audit trail: when the composer's winner changes, dump
        // every submission this frame. Future "Discord card looks wrong"
        // bug reports can be answered from the log without rerunning.
        if (openvr_pair::common::IsDebugLoggingEnabled()) {
            DiscordPresence_LogInfo(
                "[composer] winner change: %s pri=%d details='%s' state='%s' (%zu submissions)",
                winner.moduleName.c_str(), winner.update.priority,
                details.c_str(), state.c_str(), entries_.size());
            for (const auto &e : entries_) {
                const char marker = (&e == &winner) ? '*' : ' ';
                DiscordPresence_LogInfo(
                    "[composer]  %c %s pri=%d details='%s' state='%s'",
                    marker, e.moduleName.c_str(), e.update.priority,
                    e.update.details.c_str(), e.update.state.c_str());
            }
        }
        DiscordPresence_SetState(state.c_str(), details.c_str());
        lastDetails_        = details;
        lastState_          = state;
        lastLargeImageText_ = imgText;
        lastWinnerModule_   = winner.moduleName;
        lastWinnerPriority_ = winner.update.priority;
    }
}

} // namespace WKOpenVR
