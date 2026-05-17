#include "OscRouterPlugin.h"
#include "DiscordPresenceComposer.h"
#include "ShellContext.h"

#include <cstdio>
#include <memory>
#include <string>

void OscRouterPlugin::Tick(openvr_pair::overlay::ShellContext &ctx)
{
    tab_.Tick(ctx);
}

void OscRouterPlugin::DrawTab(openvr_pair::overlay::ShellContext &ctx)
{
    tab_.Draw(ctx);
}

void OscRouterPlugin::ProvidePresence(WKOpenVR::PresenceComposer &composer)
{
    const auto &s = tab_.LastStats();

    WKOpenVR::PresenceUpdate u;
    if (s.active_routes == 0 && s.packets_sent == 0) {
        // Driver not loaded, or no routes registered yet -- showing
        // "Routing OSC | 0 routes | 0 packets" was the user-confusing
        // case where Discord claimed activity that did not exist.
        u.priority = 0;
        u.details  = "OSC Router";
        u.state    = "no routes";
        composer.Submit("OSC Router", std::move(u));
        return;
    }

    // Format packet counts. Use one decimal place at the k/M boundary so
    // "1500 packets" reads as "1.5k" instead of "1k" (integer truncation
    // collapses the 1000-1999 range into a single misleading label).
    char pkts[32];
    if (s.packets_sent >= 1'000'000) {
        std::snprintf(pkts, sizeof(pkts), "%.1fM", static_cast<double>(s.packets_sent) / 1'000'000.0);
    } else if (s.packets_sent >= 1'000) {
        std::snprintf(pkts, sizeof(pkts), "%.1fk", static_cast<double>(s.packets_sent) / 1'000.0);
    } else {
        std::snprintf(pkts, sizeof(pkts), "%llu", static_cast<unsigned long long>(s.packets_sent));
    }

    std::string state = std::to_string(s.active_routes) + " routes | " + pkts + " packets";
    if (s.packets_dropped > 0) {
        state += " | " + std::to_string(s.packets_dropped) + " dropped";
    }

    u.priority = 40;
    u.details  = "Routing OSC";
    u.state    = std::move(state);

    composer.Submit("OSC Router", std::move(u));
}

namespace openvr_pair::overlay {

std::unique_ptr<FeaturePlugin> CreateOscRouterPlugin()
{
    return std::make_unique<OscRouterPlugin>();
}

} // namespace openvr_pair::overlay
