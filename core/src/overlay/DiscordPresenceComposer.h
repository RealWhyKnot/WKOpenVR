#pragma once

#include <string>
#include <vector>

namespace WKOpenVR {

// Per-module activity description submitted each tick. The composer picks
// the highest-priority submission and pushes it to DiscordPresence. When
// all submissions are at priority 0, the composer rotates through them
// once every kIdleCarouselSec seconds so each module gets a turn.
struct PresenceUpdate
{
    int         priority       = 0;   // 0=idle, 30=waiting, 40=routing, 50=active work, 100=warning, 200=pending
    std::string details;              // main activity line (up to 128 bytes)
    std::string state;                // status line (up to 128 bytes)
    std::string largeImageText;       // hover text (up to 128 bytes); "" -> composer fills a default
};

class PresenceComposer
{
public:
    // Called by each plugin's ProvidePresence once per tick.
    void Submit(const std::string &moduleName, PresenceUpdate update);

    // Called by the shell after all plugins have submitted. Picks the winner
    // and pushes to DiscordPresence_SetState only when the composed activity
    // changes or the idle carousel advances.
    void Tick();

    // Reset submission list at the start of each tick so stale entries from
    // absent / uninstalled plugins do not persist.
    void BeginFrame();

private:
    struct Entry
    {
        std::string   moduleName;
        PresenceUpdate update;
    };

    std::vector<Entry> entries_;

    // Idle carousel state.
    size_t   carouselIndex_  = 0;
    double   lastCarouselSec_  = 0.0;

    // Last values pushed to DiscordPresence, used for change detection.
    std::string lastDetails_;
    std::string lastState_;
    std::string lastLargeImageText_;

    // Last winning entry's identifying metadata. Used by the debug-mode
    // "winner change" audit log so transitions between equally-formatted
    // submissions (e.g. two modules at priority 50 with the same details)
    // are still flagged.
    std::string lastWinnerModule_;
    int         lastWinnerPriority_ = -1;
};

} // namespace WKOpenVR
