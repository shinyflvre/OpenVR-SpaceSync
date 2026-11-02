<img width="1454" height="849" alt="Screenshot 2026-08-27 092708" src="https://github.com/user-attachments/assets/b02dc130-7f84-4ab7-bef0-c6c14b4dece0" />


# SpaceSync

> **Heads up:** OpenVR-SpaceSync is a modified version of
> [OpenVR-SpaceOverride](https://github.com/Nyabsi/OpenVR-SpaceOverride) by Nyabsi (Copyright (C) 2026 Nyabsi,
> AGPL-3.0). Modifications to the original work were made beginning on 2026-08-23 by Shinyflvres.
> What changed is listed in [NOTICE.md](NOTICE.md). The license stays AGPL-3.0, see [LICENSE](LICENSE).

SpaceSync keeps your lighthouse gear (Vive/Tundra trackers, Index controllers, base stations) lined up with a SLAM headset (Galaxy XR, Quest, Pico, anything streamed through Virtual Desktop, Steam Link, ALVR and friends). It needs one tracker mounted on the headset. That tracker tells the driver how the two tracking systems relate every frame, so nothing drifts apart over time.

It is a fork of Nyabsi's [OpenVR SpaceOverride](https://github.com/Nyabsi/OpenVR-SpaceOverride), which itself grew out of pushrax's [OpenVR SpaceCalibrator](https://github.com/pushrax/OpenVR-SpaceCalibrator). Big thanks to both.

> Beta. It works well on my setup (Galaxy XR + Virtual Desktop, Vive Tracker 3.0 on the head, three body trackers, two Index controllers, four base stations). Other setups should work too, but I have not tested them all.

## Two ways to run it

**Tracker drives the headset** (this is what SpaceOverride does). After calibration the driver throws away the headset's own pose and builds it from the head tracker plus the calibrated offset. Headset and trackers then live in the same lighthouse space. Great when your streamer displays the frame exactly the way SteamVR rendered it.

**Follow SLAM HMD**. The headset keeps its own SLAM pose. The head tracker is only used to measure the offset between SLAM space and lighthouse space, and that offset is applied to every lighthouse device instead. Use this when your streamer reprojects with the headset's own tracking. A common symptom is that after a brief tracking hiccup the whole world stays rotated by a few degrees, you get black borders at the edge of the view, and your hands are no longer in front of you until you recalibrate. Follow mode prevents this. A relocalisation simply shows up as your trackers jumping once and settling again.

Both modes share the same calibration. Switching is one checkbox and does not require recalibration.

## How it compares

### Space Calibrator

Space Calibrator does a one time calibration between two tracking systems and pushes a fixed transform onto the devices that are not the headset. The two spaces slowly slide apart, so you need to recalibrate now and then. The continuous calibration forks (bd_, ArcticFox8515, hyblocker) fixed that by mounting a tracker on the headset and running the solver continuously with a rolling sample buffer in the background.

SpaceSync's follow mode uses the same general idea, but handles it differently:

* The offset is measured in closed form every frame from the calibrated rigid mount. There is no rolling Kabsch solve and no need to move around so the solver gets enough variety.

* The offset measurement is weighted by head speed and paused during fast head movement. Your trackers and controllers are still tracked live the whole time. Only the measurement of the offset waits until your head is calm again. This keeps feet and hands from swaying when you turn your head quickly.

* The remaining time offset between the headset pose and tracker pose is estimated online and compensated. Streamers often predict their poses ahead of time, so this prevents head movement from leaking into the alignment.

* Lighthouse tracking state is respected. An IMU only dead reckoned tracker pose never drives anything and never updates the offset.

* A relocalisation is corrected within about a second once your head movement is calm.

I have not benchmarked it against hyblocker's fork, so I am not making any big claims. It is a smaller and more direct implementation that came out of chasing a very specific bug.

### SpaceOverride

SpaceSync keeps SpaceOverride's tracker drives headset mode untouched and adds the follow mode on top. Beyond that, this is what changed internally:

* The driver only trusts `TrackingResult_Running_OK` poses from the head tracker. Before, a tracker that lost its base stations kept driving the headset with a frozen position and gyro only rotation. That broken data was also learned into the drift estimate.

* The drift estimate (SLAM vs tracker) is only fed by clean frames and its rotation and translation pair is always consistent.

* Config handoff between overlay and driver had a race condition because the enable flag could be written before the tracker ID. This is fixed, together with additional bounds checks.

* The overlay sends one transform per device instead of disabling and enabling it again, which could previously let a single frame through without a transform.

* The driver logs tracker state changes, drift jumps, and the learned latency offset to `spacesync_driver.log`, so there is an answer to "why did my view just move?"

## Requirements

* A lighthouse setup

* One rigid tracker on the headset (Vive Tracker 3.0 works great, Tundra is jittery on the head) when using override. It should work smoothly in Follow HMD mode though.

* A headset with its own positional tracking (SLAM)

Compatibility inherited from SpaceOverride includes PICO Connect, Virtual Desktop, ALVR, Steam Link, DisplayPort SLAM headsets (Pimax, PSVR2, Reverb G2), and VIVE Hub. SpaceSync itself is tested with Virtual Desktop and a Galaxy XR. If you run something else, tell me how it went.

## Calibration

1. Mount the tracker on the headset. It has to stay in place. If it shifts, you need to recalibrate.

2. Open SpaceSync (desktop window or SteamVR dashboard) and click the big circle.

3. If you have more than one tracker, it first asks you to move your head so it can identify the tracker mounted on the headset.

4. Follow the wizard: look left, straight, right, straight, up, straight, down, straight. Use smooth movements and take your time. You do not necessarily need to follow the wizard exactly and can speed up the sequence if you want to. The wizard mainly helps new users through calibration.

5. Done. The profile saves itself and the driver keeps running in the background. SteamVR remembers it across restarts.

If the result feels off, choose a slower calibration speed in Settings and run it again. Slow is a good default.

## Troubleshooting

**View rotated / black borders / hands not in front of me after a tracking hiccup.** Turn on Follow SLAM HMD. If you want to see what happened, look at `spacesync_driver.log` in the SteamVR folder, usually `C:\Program Files (x86)\Steam\steamapps\common\SteamVR\` (on Linux: `~/.local/share/SpaceSync/`). Lines starting with `Head tracker:` show tracker state changes, `Drift (SLAM->tracker) changed` shows the relation between the two spaces jumping, and `latency offset` is the learned time offset.

**Trackers sway when I turn my head fast (follow mode).** Hold still for a second and it should settle. The alignment is averaged over a few seconds of calm head movement, so a moment of standing still is all it needs.

**Controllers jump, then settle back.** This is expected when the headset relocalises. That is the alignment catching up. The driver runs a change detector on the alignment and applies a real jump in one go instead of sliding it in (`Drift jump compensated` in the log).

**The app says the head tracker seems to have moved.** The driver keeps refining the mount offset while you play and corrects small shifts on its own (the refined values are saved back into your profile). If that correction grows past a few degrees or centimetres, you get the yellow line under the circle. A fresh calibration is the clean fix.

**SpaceSync says the driver is unavailable.** SteamVR has to be running and the SpaceSync add on has to be enabled under SteamVR Settings > Startup / Shutdown > Manage Add ons. Overlay and driver must come from the same build. The installer takes care of that.

**My calibration feels odd.** Recalibrate with a slower speed, move smoothly, and make sure the tracker cannot wobble on the headset.

## Building

You need Visual Studio with the C++ workload (brings CMake and Ninja), the Vulkan SDK, and NSIS. Then:

```
build.bat            configure, build, copy, make the installer

build.bat nopack     build and copy only

build.bat clean      wipe the build folder first
```

The installer lands in `dev-resources\SpaceSync_Installer.exe`. Git submodules are pulled automatically if they are missing.

## Linux

SpaceSync also runs on Linux with native SteamVR (tested on Fedora with a Quest 3 over Steam Link, Index controllers and Lighthouse trackers).
The Linux port and the Ukrainian translation were made by [Lokinya](https://github.com/lokinyauv-create) with AI (Claude, Anthropic); see [NOTICE.md](NOTICE.md).

Install (SteamVR must be closed). This builds SpaceSync and installs it for your user, no root needed:

```
sudo dnf install cmake ninja-build gcc-c++ vulkan-loader-devel   # Fedora; use your distro's packages elsewhere
git submodule update --init --recursive
linux/install.sh
```

The installer copies the app to `~/.local/opt/SpaceSync`, registers the driver with SteamVR, adds a menu entry and turns on autolaunch with SteamVR. Because two calibrators moving the same devices fight each other, it switches Space Calibrator's driver off; `~/.local/opt/SpaceSync/uninstall.sh` removes SpaceSync and turns Space Calibrator back on. Running `linux/install.sh` again updates an existing install and keeps your calibration.

On Linux the profile lives in `~/.local/share/SpaceSync/profile.json` and the driver log in `~/.local/share/SpaceSync/spacesync_driver.log`. Powering base stations over Bluetooth is not available on Linux yet.

## Credits and license

* Nyabsi for OpenVR SpaceOverride, the base this fork stands on.

* pushrax (tach) for OpenVR SpaceCalibrator, the calibration math and the driver hook idea.

* bd_, ArcticFox8515 and hyblocker for showing that continuous calibration with a head tracker is the way to go.

* Fonts: Manrope and JetBrains Mono, both SIL Open Font License.

Licensed under AGPLv3, see [LICENSE](LICENSE). Commits up to and including `1cc0583` of the original project are MIT, see [LICENSE.MIT](LICENSE.MIT).
