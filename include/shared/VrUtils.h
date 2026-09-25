// SPDX-License-Identifier: AGPL-3.0-only

#pragma once

#include <format>
#include <span>
#include <string>
#include <stdexcept>

#include <openvr.h>
#include <SDL3/SDL.h>

static auto OpenVRInit(vr::EVRApplicationType type) -> void
{
    vr::EVRInitError result = {};
    VR_Init(&result, type);

    // if VRApplication_Background is specified when trying to launch
    // and SteamVR is not running, it will throw VRInitError_Init_NoServerForBackgroundApp (121)
    if (result == vr::VRInitError_Init_NoServerForBackgroundApp)
        throw std::runtime_error(std::format("This application requires SteamVR to be running to start!"));

    if (result == vr::VRInitError_Init_HmdNotFound)
        throw std::runtime_error(std::format("SteamVR was running but headset was not found."));
}

static auto OpenVRManifestInstalled(const char* appKey) -> bool
{
    return vr::VRApplications()->IsApplicationInstalled(appKey);
}

static auto OpenVRManifestInstall(const std::string& directory) -> void
{
    std::string manifestPath = directory + "\\manifest.vrmanifest";

    vr::EVRApplicationError result = vr::VRApplications()->AddApplicationManifest(manifestPath.data());
    if (result > vr::VRApplicationError_None)
        throw std::runtime_error(std::format("Failed to add manifest from \"{}\" ({})", manifestPath, static_cast<int>(result)));
}

class VrTrackedDeviceProperties {
  public:
    [[maybe_unused]] static auto FromDeviceIndex(uint32_t deviceIndex) -> VrTrackedDeviceProperties {
        return VrTrackedDeviceProperties{deviceIndex};
    }

    [[nodiscard]] auto Handle() const -> vr::TrackedDeviceIndex_t { return handle; }

    [[maybe_unused]] auto CheckConnection() const -> void {
        if (!vr::VRSystem()->IsTrackedDeviceConnected(handle))
            throw std::runtime_error("The device must be connected to use VrTrackedDeviceProperties!");
    }

    [[maybe_unused]] auto GetString(const vr::ETrackedDeviceProperty property) const -> std::string { 
        vr::ETrackedPropertyError result = {};
        std::vector<char> buffer(vr::k_unMaxPropertyStringSize);
        auto buffer_len = vr::VRSystem()->GetStringTrackedDeviceProperty(handle, property, buffer.data(), vr::k_unMaxPropertyStringSize, &result);
        if (result != vr::TrackedProp_Success || buffer_len == 0) {
            throw std::runtime_error(std::format(
                "Failed to get string prop \"{}\" for {} (err={})",
                static_cast<int>(property),
                static_cast<int>(handle),
                static_cast<int>(result)
            ));
        }

        return buffer.data();
    }

    [[maybe_unused]] auto GetBool(const vr::ETrackedDeviceProperty property) -> bool {
        vr::ETrackedPropertyError result = {};
        auto value = vr::VRSystem()->GetBoolTrackedDeviceProperty(handle, property, &result);
        if (result > vr::TrackedProp_Success)
            throw std::runtime_error(
                std::format(
                    "Failed to get bool prop \"{}\" for {} ({})",
                    static_cast<int>(property),
                    static_cast<int>(handle),
                    static_cast<int>(result)
                ));
        return value;
    }

    [[maybe_unused]] auto GetFloat(const vr::ETrackedDeviceProperty property) -> float {
        vr::ETrackedPropertyError result = {};
        auto value = vr::VRSystem()->GetFloatTrackedDeviceProperty(handle, property, &result);
        if (result > vr::TrackedProp_Success)
            throw std::runtime_error(
                std::format(
                    "Failed to get float prop \"{}\" for {} ({})",
                    static_cast<int>(property),
                    static_cast<int>(handle),
                    static_cast<int>(result)
                ));
        return value;
    }

    [[maybe_unused]] auto GetInt32(const vr::ETrackedDeviceProperty property) -> int32_t {
        vr::ETrackedPropertyError result = {};
        auto value = vr::VRSystem()->GetInt32TrackedDeviceProperty(handle, property, &result);
        if (result > vr::TrackedProp_Success)
            throw std::runtime_error(
                std::format(
                    "Failed to get int32 prop \"{}\" for {} ({})",
                    static_cast<int>(property),
                    static_cast<int>(handle),
                    static_cast<int>(result)
                ));
        return value;
    }

    // TODO: implement
    // [[maybe_unused]] auto GetArray(const vr::ETrackedDeviceProperty property) -> void { }

  private:
    explicit VrTrackedDeviceProperties(const vr::TrackedDeviceIndex_t handle) : handle{handle} {}

    vr::TrackedDeviceIndex_t handle;
};