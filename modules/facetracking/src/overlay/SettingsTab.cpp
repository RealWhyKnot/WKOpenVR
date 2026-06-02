#include "SettingsTab.h"

#include "FacetrackingPlugin.h"
#include "UiHelpers.h"

#include <imgui/imgui.h>

#include <cstring>

namespace facetracking::ui {

using namespace openvr_pair::overlay::ui;

void DrawSettingsTab(FacetrackingPlugin &plugin)
{
    FacetrackingProfile &p = plugin.profile_.current;

    // ---- Eyelid Sync ----
    DrawSectionHeading("Eyelid Sync");

    if (CheckboxWithTooltip("Eyelid Sync", &p.eyelid_sync_enabled,
            "Blends both eye openness values toward a weighted average\n"
            "to reduce asymmetric flicker from tracking noise.\n"
            "Strength 0 = no effect even when enabled.")) {
        plugin.PushConfigToDriver();
    }

    if (!p.eyelid_sync_enabled) ImGui::BeginDisabled();

    if (SliderIntWithTooltip("Sync Strength##eyelid", &p.eyelid_sync_strength,
            0, 100, "%d%%",
            "How aggressively both eyes are pulled toward the average.\n"
            "100 = fully forced equal; 0 = no correction applied.\n"
            "70 is a safe default -- covers sensor noise without\n"
            "flattening deliberate asymmetry.")) {
        plugin.PushConfigToDriver();
    }

    if (CheckboxWithTooltip("Preserve intentional winks", &p.eyelid_sync_preserve_winks,
            "Detects a deliberate one-eye wink (asymmetry > 45%% sustained\n"
            "for 120 ms with good confidence) and bypasses sync during\n"
            "it so winks survive even at high strength settings.")) {
        plugin.PushConfigToDriver();
    }

    if (!p.eyelid_sync_enabled) ImGui::EndDisabled();

    // ---- Expression Corrections ----
    DrawSectionHeading("Expression Corrections");

    if (CheckboxWithTooltip("Mouth-close compensation", &p.mouth_close_compensation_enabled,
            "Reduces JawOpen when MouthClose is also active.\n"
            "Use this when the tracker leaves the avatar's mouth slightly open\n"
            "even though the lips are meant to be closed.")) {
        plugin.PushConfigToDriver();
    }

    if (CheckboxWithTooltip("Smile opens mouth", &p.smile_mouth_open_assist_enabled,
            "Adds a small JawOpen floor as smiles get stronger.\n"
            "Use this on avatars where a broad tracked smile looks too stiff\n"
            "with the mouth fully closed.")) {
        plugin.PushConfigToDriver();
    }

    if (!p.smile_mouth_open_assist_enabled) ImGui::BeginDisabled();

    if (SliderIntWithTooltip("Smile open strength##smile_mouth_open",
            &p.smile_mouth_open_strength, 0, 100, "%d%%",
            "Maximum mouth-open assist for strong smiles.\n"
            "Lower values keep closed-mouth smiles mostly intact; higher\n"
            "values force a clearer open-mouth grin.")) {
        plugin.PushConfigToDriver();
    }

    if (!p.smile_mouth_open_assist_enabled) ImGui::EndDisabled();

    if (CheckboxWithTooltip("Auto-close idle mouth", &p.idle_mouth_auto_close_enabled,
            "Closes a small, steady JawOpen value after it persists for a\n"
            "short time with little other mouth expression.\n"
            "Use this for headset noise that appears when reclining or lying down.")) {
        plugin.PushConfigToDriver();
    }

    if (CheckboxWithTooltip("Sync brows with closed eyes", &p.eyelid_brow_sync_enabled,
            "Softly reduces brow-up shapes and adds a small brow-lowerer\n"
            "hint when eyelids are closed.\n"
            "Use this when blinks or reclined poses make brows and lids look\n"
            "out of sync on a specific avatar.")) {
        plugin.PushConfigToDriver();
    }

    if (!p.eyelid_brow_sync_enabled) ImGui::BeginDisabled();

    if (SliderIntWithTooltip("Brow sync strength##eyelid_brow",
            &p.eyelid_brow_sync_strength, 0, 100, "%d%%",
            "How strongly closed eyelids influence brow shapes.\n"
            "Lower values preserve expressive brows; higher values make\n"
            "blinks and closed-eye poses more tightly coordinated.")) {
        plugin.PushConfigToDriver();
    }

    if (!p.eyelid_brow_sync_enabled) ImGui::EndDisabled();

    // ---- Vergence Lock ----
    DrawSectionHeading("Vergence Lock");

    if (CheckboxWithTooltip("Vergence Lock", &p.vergence_lock_enabled,
            "Reconstructs a shared focus point from both gaze rays each\n"
            "frame (skew-line midpoint) and nudges both eyes toward it.\n"
            "Reduces the jitter and crossed-eye artefacts that appear\n"
            "when each eye's tracker drifts independently.")) {
        plugin.PushConfigToDriver();
    }

    if (!p.vergence_lock_enabled) ImGui::BeginDisabled();

    if (SliderIntWithTooltip("Lock Strength##vergence", &p.vergence_lock_strength,
            0, 100, "%d%%",
            "How much each eye's gaze is pulled toward the computed\n"
            "focus point. Start at 50 and increase if you still see\n"
            "crossed-eye artefacts; lower if the focus point feels\n"
            "sluggish on fast saccades.")) {
        plugin.PushConfigToDriver();
    }

    if (!p.vergence_lock_enabled) ImGui::EndDisabled();

    // ---- Output ----
    DrawSectionHeading("Output");

    if (CheckboxWithTooltip("OSC (VRChat)", &p.output_osc_enabled,
            "Sends /avatar/parameters/* to the OSC router, which forwards\n"
            "them to VRChat. Target host and port are configured on the\n"
            "OSC Router tab. Uncheck to stop all FT OSC output.")) {
        plugin.PushConfigToDriver();
    }

}

} // namespace facetracking::ui
