# Overlay UI library (`openvr_pair_overlay_ui`)

Shared widgets, layout, and theming for the WKOpenVR overlay. Every feature
module's tab is built from these components so the whole app looks and behaves
the same way, and so a fix or a theme change lands everywhere at once.

Namespace: `openvr_pair::overlay::ui`. Include `UiCore.h` to pull in the whole
set, or the individual headers below.

## How modules link it

Module overlay libraries do **not** link this library directly. They get the
headers through the `core/src/overlay` include path and call the functions; the
symbols are resolved when the final `WKOpenVR.exe` (`OpenVRPairOverlay`) links
`openvr_pair_overlay_ui`. So adding a component is a header declaration plus a
body in the matching `.cpp` here -- no module build file needs to change.

## Components

`Theme.h`
- `GetPalette()` / `SemanticPalette` -- semantic color tokens (statusOk,
  statusPending, statusError, statusWarn, statusInfo, statusIdle, banners,
  plot accents, windowBg).
- `StatusColor(StatusTone)` -- a token as an `ImVec4`.
- `SetTheme(ThemeId)`, `GetCurrentThemeId()`, `InitThemeFromDisk()`. Themes:
  Modern (default), Legacy, Dark, Light, High Contrast, Pure Dark.

`UiLayout.h`
- `ApplyOverlayStyle()` -- one-time global style metrics (padding, rounding).
- `DrawPanel(name, body)` / `PanelScope` -- a titled bordered group.
- `DrawCard(title, tone, body)` -- a panel whose title is tinted by a status
  tone; use it for state-bearing sections (Idle = plain panel).
- `TabBarScope`, `DrawTabItem`, `DrawScrollableTabItem`, `ChildScope`.

`UiControls.h`
- `CheckboxWithTooltip`, `SliderIntWithTooltip`, `SliderFloatWithTooltip`,
  `RadioButtonWithTooltip`, `DrawHelpMarker`.
- `DrawStatusText(text, tone)`, `DrawStatusDot`, `StatusBadge(text, tone)`,
  `DrawEmptyState`, `RightAlignText`.
- Banners: `DrawErrorBanner`, `DrawWaitingBanner`, `DrawInfoBanner`,
  `DrawSectionHeading`.
- `ActionButton` + `DrawActionRow`, `DisabledSection`, `ScopedStyleColor`,
  `ScopedStyleColors`, clipboard + byte/age formatters.

`UiTables.h`
- `TableScope`, `SettingTableScope`, `DrawSettingTable`, `SettingRow`.
- `SetupStretchColumn`, `SetupFixedColumn`, `DrawStatusCell`,
  `DrawKeyValueRow`.
- `SetCellToneBg(tone)` / `ToneCellBgColor(tone)` -- theme-derived cell fill.

`UiResponsive.h`
- `ResponsiveColumnsScope(id, columns, minColumnWidth)` -- an equal-width grid
  that collapses toward one column as the window narrows.
- `FlowRowScope` -- a row of similarly-sized chips/buttons that wraps instead
  of overflowing.
- `UiResponsiveLogic.h` holds the pure layout math, kept ImGui-free for tests.

## Conventions

Color: never hardcode an `ImVec4` or ARGB literal for state. Route it through
`StatusColor(tone)`, a `GetPalette()` token, or `SetCellToneBg(tone)`. A theme
switch then recolors the whole UI without touching any call site.

Pure logic: keep tone/label/classification helpers in an ImGui-free header
(for example `UiResponsiveLogic.h`, `PhantomUiLogic.h`) so they can be unit
tested and so the header doubles as a compile check that they stay ImGui-free.

Layout: one fullscreen window via `GetMainViewport()` -- no docking APIs. The
overlay runs at a fixed 1200x780 in the VR dashboard and resizable on the
desktop, so components must read well from about 640px wide upward. Prefer
stretch columns, `DrawPanel`, and `ResponsiveColumnsScope` over fixed
`SameLine(x)` offsets or fixed child widths that overflow a narrow window.

Text: a tab gets at most one line of context; detail belongs in per-control
tooltips, not a paragraph at the top.

## Pitfall: do not fight the user every frame

Let ImGui own interactive state and read it back; do not force the state from
your own variable on every frame.

- Tabs: pass `ImGuiTabItemFlags_SetSelected` only on the one frame you change
  selection programmatically (a next-tab button, a default), then stop. Forcing
  it every frame to mirror an externally-tracked key overrides the user's click
  and the selection oscillates back. Otherwise read the choice via
  `if (tab) selected = key;` and leave the flag off.
- Same rule for `SetKeyboardFocusHere`, `ActivateItem`, and `SetScrollHereY`:
  one-shot on the frame you mean it, never unconditionally.
- Text fields: hydrate the buffer once (or when the selection changes), let the
  user edit, and commit on `IsItemDeactivatedAfterEdit`. Re-copying a model
  string into the buffer every frame is only safe if you also write edits back
  the same frame so the two never diverge.

## Adding a component

Promote a pattern to this library once two or more modules need it. Put the
declaration in the matching header and the body in the matching `.cpp` (both
already in the `openvr_pair_overlay_ui` target). Add a render smoke test in
`tests/ui/test_ui_core.cpp`, and if there is pure decision logic, add it to an
ImGui-free header with a unit test.
