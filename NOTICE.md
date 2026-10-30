# Notice of modification

OpenVR-SpaceSync is a modified version of OpenVR-SpaceOverride by Nyabsi
(https://github.com/Nyabsi/OpenVR-SpaceOverride). Modifications to the original work were made
beginning on 2026-08-23 by Shinyflvres. The original project is licensed under the GNU Affero
General Public License v3.0 (AGPL-3.0-only), and so is this one. Nothing about the license changed.

- Original work: OpenVR-SpaceOverride, Copyright (C) 2026 Nyabsi
- Modified by: Shinyflvres (https://github.com/shinyflvre/OpenVR-SpaceSync)
- First modified release: 2026-08-23, work continues since then
- License: AGPL-3.0-only, see LICENSE

OpenVR-SpaceOverride itself is based on OpenVR-SpaceCalibrator by pushrax (MIT, see LICENSE.MIT).

## What I changed

Starting point was OpenVR-SpaceOverride commit `6604e42`. Since then:

- Driver: the head tracker only counts while it reports `TrackingResult_Running_OK`, the drift
  estimate only learns from clean frames and keeps its rotation and translation consistent,
  head motion is weighted down while measuring, a leftover latency between headset and tracker
  is learned on the fly, a couple of races and missing bounds checks got fixed, and the driver
  now logs tracker state changes and drift jumps.
- New "Follow SLAM HMD" mode: the headset keeps its own SLAM pose and the lighthouse devices
  (head tracker included) get re-aligned to it every frame.
- Overlay: one transform message per device instead of disable/enable, a new UI (Theme.cpp and
  UserInterface.cpp, with Manrope and JetBrains Mono embedded), a UI scale setting, a calibration
  wizard, descriptions under every setting, and cancel/result handling for the calibration.
- Rebrand to SpaceSync: exe, driver, IPC pipe, registry key (old OpenVR-SpaceOverride profiles are
  still picked up), installer (removes an old OpenVR-SpaceOverride install), build.bat, icon, README.

Every source file I touched or added has a "Modified by" or "Added by" line under its SPDX header.
The full history is in this repo's git log.

## Linux port (Lokinya)

The `linux-port` branch is a Linux port of SpaceSync made by Lokinya (https://github.com/lokinyauv-create),
beginning on 2026-10-30. The work was done with AI assistance (Claude, Anthropic). It stays AGPL-3.0-only.
All of the calibration, alignment, smoothing and base station lock logic is SpaceSync's, unchanged.
The Windows code paths are kept as they were, behind `#ifdef _WIN32`.

- Driver: vtable-slot hooks instead of MinHook, an abstract unix socket instead of the named pipe,
  small Win32 stand-ins in `include/shared/Platform.h`, log file in `~/.local/share/SpaceSync`.
- Overlay: unix socket IPC client, profile in `~/.local/share/SpaceSync/profile.json` instead of the
  registry, single instance via `flock`, SteamVR handover, SDL3 audio for the voice cues, Linux paths
  and CJK fonts, Wayland/X11 Vulkan surface extensions.
- Base station power over Bluetooth is not available on Linux yet (`src/overlay/Lighthouse_linux.cpp` stub).
- Ukrainian language: all UI strings (`src/overlay/Localization_uk.inc`), Ukrainian voice cues in
  `src/sound/uk` (generated with Microsoft Edge TTS, voice uk-UA-PolinaNeural), Cyrillic glyphs from the
  Manrope and JetBrains Mono fonts in `resources/fonts` (SIL OFL 1.1, licences included).
- `linux/install.sh` and `linux/uninstall.sh`: per-user install, driver registration, and Space Calibrator
  switched off while SpaceSync is installed (restored on uninstall).
