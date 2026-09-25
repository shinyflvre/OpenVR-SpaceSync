// SPDX-License-Identifier: AGPL-3.0-only
// Modified by Shinyflvres, 2026-08-23. Part of SpaceSync, a modified version of OpenVR-SpaceOverride by Nyabsi (AGPL-3.0). See NOTICE.md

#ifdef _WIN32
#include <Windows.h>
#endif

#include <stdio.h>
#include <stdlib.h>
#include <direct.h>

#include <string>
#include <thread>
#include <chrono>

#include <imgui.h>
#include <backends/imgui_impl_sdl3.h>
#include <backends/imgui_impl_vulkan.h>

#include <SDL3/SDL.h>
#include <SDL3/SDL_mouse.h>
#include <SDL3/SDL_vulkan.h>

#include <openvr.h>

#include "VulkanRenderer.h"
#include "VulkanUtils.h"

#include "ImGuiWindow.h"

#include "VrOverlay.h"
#include "VrUtils.h"

#include "imgui_impl_openvr.h"

#include "Calibration.h"
#include "Configuration.h"
#include "UserInterface.h"
#include "Sound.h"
#include "Lighthouse.h"

#ifdef _WIN32
extern "C" __declspec(dllexport) unsigned long NvOptimusEnablement = 0x00000001;
extern "C" __declspec(dllexport) unsigned long AmdPowerXpressRequestHighPerformance = 0x00000001;
#endif

using namespace std::chrono_literals;

static VulkanRenderer* g_vulkanRenderer = new VulkanRenderer();
static ImGuiWindow* g_imGuiWindow = new ImGuiWindow();
static VrOverlay* g_overlay = new VrOverlay();

static vr::VROverlayHandle_t g_notifyOverlayHandle = vr::k_ulOverlayHandleInvalid;

static uint64_t g_last_frame_time = SDL_GetTicksNS();
static float g_hmd_refresh_rate = 90.0f;
static bool g_ticking = true;
static bool g_dashboard_was_visible = false;
static bool g_tracking_lost = false;
static uint64_t g_tracking_lost_time = 0;

#define APP_KEY     "Shinyflvres.SpaceSync"
#define APP_NAME    "SpaceSync"
#define NOTIFY_KEY  "Shinyflvres.SpaceSyncNotifier"

#define WIN_WIDTH   1080
#define WIN_HEIGHT  700

static auto HandleCommandLine(int argc, char** argv) -> void;

static auto UpdateApplicationRefreshRate() -> void
{
    try {
        auto hmd_properties = VrTrackedDeviceProperties::FromDeviceIndex(vr::k_unTrackedDeviceIndex_Hmd);
        hmd_properties.CheckConnection();
        g_hmd_refresh_rate = hmd_properties.GetFloat(vr::Prop_DisplayFrequency_Float);
    }
    catch (std::exception& ex) {
        printf("%s\n\n", ex.what());
    }
}

static auto ActivateMultipleDrivers() -> void
{
    vr::EVRSettingsError settingsError = vr::VRSettingsError_None;
    bool enabled = vr::VRSettings()->GetBool(vr::k_pch_SteamVR_Section, vr::k_pch_SteamVR_ActivateMultipleDrivers_Bool, &settingsError);

    if (settingsError != vr::VRSettingsError_None) {
        std::string err = "Could not read \"" + std::string(vr::k_pch_SteamVR_ActivateMultipleDrivers_Bool) + "\" setting: "
            + vr::VRSettings()->GetSettingsErrorNameFromEnum(settingsError);
        throw std::runtime_error(err);
    }

    if (!enabled) {
        vr::VRSettings()->SetBool(vr::k_pch_SteamVR_Section, vr::k_pch_SteamVR_ActivateMultipleDrivers_Bool, true, &settingsError);
        if (settingsError != vr::VRSettingsError_None) {
            std::string err = "Could not set \"" + std::string(vr::k_pch_SteamVR_ActivateMultipleDrivers_Bool) + "\" setting: "
                + vr::VRSettings()->GetSettingsErrorNameFromEnum(settingsError);
            throw std::runtime_error(err);
        }
        fprintf(stderr, "Enabled \"%s\" setting\n", vr::k_pch_SteamVR_ActivateMultipleDrivers_Bool);
    }
}

static auto CreateNotificationOverlay() -> void
{
    vr::VROverlay()->CreateOverlay(NOTIFY_KEY, "SpaceSync Notification", &g_notifyOverlayHandle);

    vr::HmdMatrix34_t m = {
        1.0f, 0.0f, 0.0f,  0.0f,
        0.0f, 1.0f, 0.0f,  0.0f,
        0.0f, 0.0f, 1.0f, -2.0f
    };

    vr::VROverlay()->SetOverlayTransformTrackedDeviceRelative(g_notifyOverlayHandle, vr::k_unTrackedDeviceIndex_Hmd, &m);
}

static auto ShowNotification(const char* text) -> vr::VRNotificationId
{
    if (g_notifyOverlayHandle == vr::k_ulOverlayHandleInvalid)
        return 0;

    vr::VRNotificationId id = 0;
    vr::VRNotifications()->CreateNotification(
        g_notifyOverlayHandle, 0, vr::EVRNotificationType_Persistent,
        text, vr::EVRNotificationStyle_None, nullptr, &id
    );

    return id;
}

int main(int argc, char** argv)
{
#ifdef _WIN32
    ShowWindow(GetConsoleWindow(), SW_HIDE);
#endif
    HandleCommandLine(argc, argv);

    try {
        OpenVRInit(vr::VRApplication_Background);
        ActivateMultipleDrivers();
    }
    catch (std::exception& ex) {
#ifdef _WIN32
        MessageBoxA(NULL, ex.what(), APP_NAME, MB_OK);
#else
        printf("%s\n\n", ex.what());
#endif
        return EXIT_FAILURE;
    }

    UpdateApplicationRefreshRate();

    try {
        if (!OpenVRManifestInstalled(APP_KEY)) OpenVRManifestInstall();
    }
    catch (std::exception& ex) {
#ifdef _WIN32
        MessageBoxA(NULL, ex.what(), APP_NAME, MB_OK);
#else
        printf("%s\n\n", ex.what());
#endif
        return EXIT_FAILURE;
    }

    try {
        g_overlay->Create(vr::VROverlayType_Dashboard, APP_KEY, APP_NAME);

        std::string thumbnail_path = SDL_GetBasePath();
        thumbnail_path += "\\icon.png";
        g_overlay->SetThumbnail(thumbnail_path);

        g_overlay->SetInputMethod(vr::VROverlayInputMethod_Mouse);
        g_overlay->SetWidth(3.0f);

        g_overlay->EnableFlag(vr::VROverlayFlags_SendVRDiscreteScrollEvents);
        g_overlay->EnableFlag(vr::VROverlayFlags_EnableClickStabilization);

        CreateNotificationOverlay();
    }
    catch (std::exception& ex) {
#ifdef _WIN32
        MessageBoxA(NULL, ex.what(), APP_NAME, MB_OK);
#else
        printf("%s\n\n", ex.what());
#endif
        return EXIT_FAILURE;
    }

    sound::Init();
    lighthouse::Init();
    if (!SDL_Init(SDL_INIT_VIDEO)) {
#ifdef _WIN32
        MessageBoxA(NULL, SDL_GetError(), APP_NAME, MB_OK);
#else
        printf("%s\n\n", SDL_GetError());
#endif
        return EXIT_FAILURE;
    }

    try {
        g_vulkanRenderer->Initialize();
        g_imGuiWindow->Initialize(g_vulkanRenderer, g_overlay, APP_NAME, WIN_WIDTH, WIN_HEIGHT);
        g_imGuiWindow->Show();
    } catch (std::exception& ex) {
#ifdef _WIN32
        MessageBoxA(NULL, ex.what(), APP_NAME, MB_OK);
#else
        printf("%s\n\n", ex.what());
#endif
        return EXIT_FAILURE;
    }

    try {
        InitCalibrator();
        LoadProfile(CalCtx);
    }
    catch (std::exception& ex) {
#ifdef _WIN32
        MessageBoxA(NULL, ex.what(), APP_NAME, MB_OK);
#else
        printf("%s\n\n", ex.what());
#endif
        return EXIT_FAILURE;
    }

    SDL_Event event = {};
    vr::VREvent_t vr_event = {};

    while (g_ticking)
    {
        while (SDL_PollEvent(&event))
        {
            ImGui_ImplSDL3_ProcessEvent(&event);

            if (event.type == SDL_EVENT_WINDOW_MINIMIZED && event.window.windowID == SDL_GetWindowID(g_imGuiWindow->Window()))
                g_imGuiWindow->SetMinimizedFromEvent(true);
            if (event.type == SDL_EVENT_WINDOW_RESTORED && event.window.windowID == SDL_GetWindowID(g_imGuiWindow->Window()))
                g_imGuiWindow->SetMinimizedFromEvent(false);
            if (event.type == SDL_EVENT_WINDOW_CLOSE_REQUESTED && event.window.windowID == SDL_GetWindowID(g_imGuiWindow->Window()))
                g_imGuiWindow->Hide();
            if (event.type == SDL_EVENT_WINDOW_PIXEL_SIZE_CHANGED && event.window.windowID == SDL_GetWindowID(g_imGuiWindow->Window()))
                g_imGuiWindow->Resize(g_vulkanRenderer, event.window.data1, event.window.data2);
            if (event.type == SDL_EVENT_QUIT)
                g_ticking = false;
        }

        while (vr::VROverlay()->PollNextOverlayEvent(g_overlay->Handle(), &vr_event, sizeof(vr_event)))
        {
            ImGui_ImplOpenVR_ProcessOverlayEvent(vr_event);

            switch (vr_event.eventType)
            {
                case vr::VREvent_PropertyChanged:
                {
                    // Some drivers such as lighthouse or vrlink are capable of changing
                    // vr::Prop_DisplayFrequency_Float without restarting SteamVR
                    if (vr_event.data.property.prop == vr::Prop_DisplayFrequency_Float) {
                        UpdateApplicationRefreshRate();
                    }
                    break;
                }
                case vr::VREvent_TrackedDeviceUserInteractionStarted:
                {
                    /*
                    auto activityLevel = vr::VRSystem()->GetTrackedDeviceActivityLevel(CalCtx.targetID);
                    if ((activityLevel == vr::k_EDeviceActivityLevel_UserInteraction || activityLevel == vr::k_EDeviceActivityLevel_UserInteraction_Timeout) && g_tracking_lost) {
                        if (SDL_GetTicks() - g_tracking_lost_time >= (30 * 1000)) {
                            CalCtx.Clear();
                            std::thread([]() {
                                std::this_thread::sleep_for(3000ms);
                                CalCtx.notificationId = ShowNotification("Tracking lost - Currently recalibrating, please follow the calibration instructions.\n\nLook left, center, right, center, up, center, down, center - repeated depending on your calibration speed.");
                                StartCalibration();
                            }).detach();
                        }
                        g_tracking_lost = false;
                    }
                    */
                    break;
                }
                case vr::VREvent_TrackedDeviceUserInteractionEnded:
                {
                    /*
                    auto activityLevel = vr::VRSystem()->GetTrackedDeviceActivityLevel(CalCtx.targetID);
                    if (activityLevel == vr::k_EDeviceActivityLevel_Idle && CalCtx.enabled && !g_tracking_lost) {
                        g_tracking_lost = true;
                        g_tracking_lost_time = SDL_GetTicks();
                    }
                    */
                    break;
                }
                case vr::VREvent_Quit:
                    [[fallthrough]];
                case vr::VREvent_ProcessQuit:
                    [[fallthrough]];
                case vr::VREvent_QuitAcknowledged:
                    [[fallthrough]];
                case vr::VREvent_DriverRequestedQuit:
                    [[fallthrough]];
                case vr::VREvent_RestartRequested:
                {
                    // CalCtx.Clear();
                    vr::VRSystem()->AcknowledgeQuit_Exiting();
                    g_ticking = false;
                    break;
                }
            }
        }

        const bool dashboardVisible = vr::VROverlay()->IsActiveDashboardOverlay(g_overlay->Handle());
        if (dashboardVisible != g_dashboard_was_visible) {
            if (dashboardVisible)
                g_imGuiWindow->Hide();
            else
                g_imGuiWindow->Show();
            g_dashboard_was_visible = dashboardVisible;
        }

        const double time = static_cast<double>(SDL_GetTicks()) / 1000.0;
        CalibrationTick(time);

        g_imGuiWindow->Draw(dashboardVisible);
        ImDrawData* draw_data = ImGui::GetDrawData();

        g_vulkanRenderer->Render(draw_data);

        if (dashboardVisible)
            g_vulkanRenderer->SubmitOverlay(g_overlay);

        const bool is_minimized = g_imGuiWindow->Shown() && g_imGuiWindow->Minimized();
        g_imGuiWindow->WindowData()->is_minimized = is_minimized;

        if (g_imGuiWindow->Shown() && !is_minimized) {
            if (g_vulkanRenderer->ShouldRebuildSwapchain()) {
                ImGui_ImplVulkan_SetMinImageCount(g_vulkanRenderer->MinimumConcurrentImageCount());
                g_vulkanRenderer->SetupSwapchain(g_imGuiWindow->WindowData(), g_imGuiWindow->Width(), g_imGuiWindow->Height());
            }
            g_vulkanRenderer->BlitToWindow(g_imGuiWindow->WindowData());
            g_vulkanRenderer->Present(g_imGuiWindow->WindowData());
        }

        const uint64_t target_time_ns = static_cast<uint64_t>(1'000'000'000.0 / g_hmd_refresh_rate);
        uint64_t elapsed_ns = SDL_GetTicksNS() - g_last_frame_time;

        if (elapsed_ns < target_time_ns)
        {
            vr::VROverlay()->WaitFrameSync(static_cast<uint32_t>((target_time_ns - elapsed_ns) / 1'000'000));

            elapsed_ns = SDL_GetTicksNS() - g_last_frame_time;
            if (elapsed_ns < target_time_ns)
                SDL_DelayPrecise(target_time_ns - elapsed_ns);
        }

        g_last_frame_time = SDL_GetTicksNS();
    }

    SaveProfile(CalCtx);

    VkResult vk_result = vkDeviceWaitIdle(g_vulkanRenderer->Device());
    VK_VALIDATE_RESULT(vk_result);

    g_vulkanRenderer->DestroyRenderTarget();
    g_vulkanRenderer->DestroyWindow(g_imGuiWindow->WindowData());
    g_imGuiWindow->Destroy(g_vulkanRenderer);
    g_vulkanRenderer->Destroy();

    ImGui::DestroyContext();

    lighthouse::Shutdown();
    sound::Shutdown();
    SDL_Quit();
    vr::VR_Shutdown();

    return 0;
}

static auto HandleCommandLine(int argc, char** argv) -> void
{
    if (argc < 2)
        return;

    const std::string arg = argv[1];

    char cwd[1024] = { 0 };
    _getcwd(cwd, sizeof(cwd));

    if (arg == "-openvrpath")
    {
        auto vrErr = vr::VRInitError_None;
        vr::VR_Init(&vrErr, vr::VRApplication_Utility);
        if (vrErr == vr::VRInitError_None)
        {
            char runtimePath[1024] = { 0 };
            unsigned int pathLen = 0;
            vr::VR_GetRuntimePath(runtimePath, sizeof(runtimePath), &pathLen);

            printf("%s", runtimePath);
            vr::VR_Shutdown();
            exit(0);
        }
        fprintf(stderr, "Failed to initialize OpenVR: %s\n", vr::VR_GetVRInitErrorAsEnglishDescription(vrErr));
        vr::VR_Shutdown();
        exit(-2);
    }
    else if (arg == "-installmanifest")
    {
        auto vrErr = vr::VRInitError_None;
        vr::VR_Init(&vrErr, vr::VRApplication_Utility);
        if (vrErr == vr::VRInitError_None)
        {
            if (vr::VRApplications()->IsApplicationInstalled(APP_KEY))
            {
                char oldWd[1024] = { 0 };
                auto vrAppErr = vr::VRApplicationError_None;
                vr::VRApplications()->GetApplicationPropertyString(APP_KEY, vr::VRApplicationProperty_WorkingDirectory_String, oldWd, sizeof(oldWd), &vrAppErr);
                if (vrAppErr == vr::VRApplicationError_None)
                {
                    std::string oldManifest = std::string(oldWd) + "\\manifest.vrmanifest";
                    vr::VRApplications()->RemoveApplicationManifest(oldManifest.c_str());
                }
            }
            std::string manifestPath = std::string(cwd) + "\\manifest.vrmanifest";
            auto vrAppErr = vr::VRApplications()->AddApplicationManifest(manifestPath.c_str());
            if (vrAppErr != vr::VRApplicationError_None)
                fprintf(stderr, "Failed to add manifest: %s\n", vr::VRApplications()->GetApplicationsErrorNameFromEnum(vrAppErr));
            else
                vr::VRApplications()->SetApplicationAutoLaunch(APP_KEY, true);

            vr::VR_Shutdown();
            exit(0);
        }
        fprintf(stderr, "Failed to initialize OpenVR: %s\n", vr::VR_GetVRInitErrorAsEnglishDescription(vrErr));
        vr::VR_Shutdown();
        exit(-2);
    }
    else if (arg == "-removemanifest")
    {
        auto vrErr = vr::VRInitError_None;
        vr::VR_Init(&vrErr, vr::VRApplication_Utility);
        if (vrErr == vr::VRInitError_None)
        {
            if (vr::VRApplications()->IsApplicationInstalled(APP_KEY))
            {
                std::string manifestPath = std::string(cwd) + "\\manifest.vrmanifest";
                vr::VRApplications()->RemoveApplicationManifest(manifestPath.c_str());
            }
            vr::VR_Shutdown();
            exit(0);
        }
        fprintf(stderr, "Failed to initialize OpenVR: %s\n", vr::VR_GetVRInitErrorAsEnglishDescription(vrErr));
        vr::VR_Shutdown();
        exit(-2);
    }
    else if (arg == "-activatemultipledrivers")
    {
        int ret = -2;
        auto vrErr = vr::VRInitError_None;
        vr::VR_Init(&vrErr, vr::VRApplication_Utility);
        if (vrErr == vr::VRInitError_None)
        {
            try {
                ActivateMultipleDrivers();
                ret = 0;
            }
            catch (std::runtime_error& e) {
                fprintf(stderr, "%s\n", e.what());
            }
        }
        else
        {
            fprintf(stderr, "Failed to initialize OpenVR: %s\n", vr::VR_GetVRInitErrorAsEnglishDescription(vrErr));
        }
        vr::VR_Shutdown();
        exit(ret);
    }
}
