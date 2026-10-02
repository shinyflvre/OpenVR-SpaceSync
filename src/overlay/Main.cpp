// SPDX-License-Identifier: AGPL-3.0-only
// Modified by Shinyflvres, 2026-08-23. Part of SpaceSync, a modified version of OpenVR-SpaceOverride by Nyabsi (AGPL-3.0). See NOTICE.md

#ifdef _WIN32
#include <Windows.h>
#endif

#include <stdio.h>
#include <stdlib.h>
#include <direct.h>
#ifdef _WIN32
#include <TlHelp32.h>
#endif

#include <string>
#include <thread>
#include <chrono>
#include <fstream>
#include <sstream>
#include <algorithm>
#include <functional>

#include <picojson.h>

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
#include "Theme.h"
#include "Sound.h"
#include "Lighthouse.h"

static bool g_desktopPreview = false;
static bool g_desktopForced = false;
static bool g_handoverLaunch = false;
static bool g_handoverToVrInstance = false;
static bool g_shutdownScreen = false;
static bool g_exitStandbyDone = false;
static uint64_t g_shutdownStart = 0;
static uint64_t g_shutdownBudgetMs = 0;
static bool g_ticking = true;
static bool g_dashboardActive = false;

static void MarkExiting();

static void BeginClose()
{
    if (g_shutdownScreen)
        return;
#ifdef _WIN32
    MarkExiting();
#endif
    size_t stationCount = lighthouse::Stations().size();
    if (!g_desktopForced && !g_handoverToVrInstance && CalCtx.dynamicBasestationPower && stationCount > 0)
    {
        lighthouse::Note("close requested (X), showing standby screen");
        g_shutdownScreen = true;
        g_shutdownStart = SDL_GetTicks();
        g_shutdownBudgetMs = (uint64_t)stationCount * 8000 + 5000;
        lighthouse::BeginStandbyAll();
    }
    else
    {
        lighthouse::Note("close requested (X), shutting down");
        g_ticking = false;
    }
}
#ifdef _WIN32
static HANDLE g_showWindowEvent = NULL;
static HANDLE g_instanceMutex = NULL;
static HANDLE g_handoverEvent = NULL;
static HANDLE g_desktopMarker = NULL;
static HANDLE g_exitingMarker = NULL;

static bool NamedEventExists(const char* name)
{
    HANDLE h = OpenEventA(SYNCHRONIZE, FALSE, name);
    if (!h)
        return false;
    CloseHandle(h);
    return true;
}

static void SignalNamedEvent(const char* name)
{
    HANDLE h = OpenEventA(EVENT_MODIFY_STATE, FALSE, name);
    if (!h)
        return;
    SetEvent(h);
    CloseHandle(h);
}

static void MarkExiting()
{
    if (!g_exitingMarker)
        g_exitingMarker = CreateEventA(NULL, TRUE, TRUE, "SpaceSyncExiting");
}

static bool SteamVRProcessRunning()
{
    HANDLE snap = CreateToolhelp32Snapshot(TH32CS_SNAPPROCESS, 0);
    if (snap == INVALID_HANDLE_VALUE)
        return false;
    PROCESSENTRY32 entry = {};
    entry.dwSize = sizeof(entry);
    bool found = false;
    if (Process32First(snap, &entry))
    {
        do
        {
            if (_stricmp(entry.szExeFile, "vrserver.exe") == 0)
            {
                found = true;
                break;
            }
        } while (Process32Next(snap, &entry));
    }
    CloseHandle(snap);
    return found;
}

static bool AcquireInstance()
{
    HANDLE mutex = CreateMutexA(NULL, FALSE, "SpaceSyncSingleInstance");
    if (!mutex)
        return true;

    DWORD w = WaitForSingleObject(mutex, 0);
    bool owner = w == WAIT_OBJECT_0 || w == WAIT_ABANDONED;
    if (!owner)
    {
        bool desktopRunning = NamedEventExists("SpaceSyncDesktopInstance");
        bool exiting = NamedEventExists("SpaceSyncExiting");
        bool takeOver = (!g_desktopPreview && desktopRunning) || g_handoverLaunch || exiting;
        if (takeOver)
        {
            if (!g_desktopPreview && desktopRunning)
            {
                lighthouse::Note("steamvr instance: asking the desktop instance to hand over");
                SignalNamedEvent("SpaceSyncHandover");
            }
            for (int i = 0; i < 240 && !owner; i++)
            {
                w = WaitForSingleObject(mutex, 250);
                owner = w == WAIT_OBJECT_0 || w == WAIT_ABANDONED;
                if (!owner && !NamedEventExists("SpaceSyncDesktopInstance") && !NamedEventExists("SpaceSyncExiting"))
                    break;
            }
        }
    }

    if (!owner)
    {
        lighthouse::Note("second launch: another instance is running, bringing it to front");
        SignalNamedEvent("SpaceSyncShowWindow");
        CloseHandle(mutex);
        return false;
    }

    g_instanceMutex = mutex;
    g_showWindowEvent = CreateEventA(NULL, FALSE, FALSE, "SpaceSyncShowWindow");
    if (g_desktopPreview)
    {
        g_desktopMarker = CreateEventA(NULL, TRUE, TRUE, "SpaceSyncDesktopInstance");
        g_handoverEvent = CreateEventA(NULL, FALSE, FALSE, "SpaceSyncHandover");
    }
    return true;
}

static void ReleaseInstance()
{
    if (g_instanceMutex)
    {
        ReleaseMutex(g_instanceMutex);
        CloseHandle(g_instanceMutex);
        g_instanceMutex = NULL;
    }
}

static bool AutoLaunchInitialized()
{
    DWORD value = 0, size = sizeof(value);
    return RegGetValueA(HKEY_CURRENT_USER, "Software\\SpaceSync", "AutoLaunchInitialized", RRF_RT_REG_DWORD, NULL, &value, &size) == ERROR_SUCCESS && value == 1;
}

static void MarkAutoLaunchInitialized()
{
    DWORD value = 1;
    RegSetKeyValueA(HKEY_CURRENT_USER, "Software\\SpaceSync", "AutoLaunchInitialized", REG_DWORD, &value, sizeof(value));
}
#endif

static std::string ExeDirectory()
{
#ifdef _WIN32
    char path[1024] = {};
    GetModuleFileNameA(nullptr, path, sizeof(path));
    std::string s(path);
    size_t slash = s.find_last_of("\\/");
    return slash == std::string::npos ? std::string(".") : s.substr(0, slash);
#else
    return ".";
#endif
}

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

static void RenderFrame()
{
    const bool overlayValid = !g_desktopPreview && vr::VROverlay() && g_overlay->Handle() != vr::k_ulOverlayHandleInvalid;
    const bool dashboardVisible = overlayValid && vr::VROverlay()->IsActiveDashboardOverlay(g_overlay->Handle());

    g_dashboardActive = dashboardVisible;
    g_imGuiWindow->Draw(dashboardVisible, 0.0f);
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
}

static bool SDLCALL LiveResizeWatch(void*, SDL_Event* event)
{
    if (!g_ticking || !g_imGuiWindow || !g_imGuiWindow->Window())
        return true;
    if (event->window.windowID != SDL_GetWindowID(g_imGuiWindow->Window()))
        return true;

    if (event->type == SDL_EVENT_WINDOW_PIXEL_SIZE_CHANGED)
    {
        ImGui_ImplSDL3_ProcessEvent(event);
        g_imGuiWindow->Resize(g_vulkanRenderer, event->window.data1, event->window.data2);
        RenderFrame();
    }
    else if (event->type == SDL_EVENT_WINDOW_EXPOSED)
    {
        RenderFrame();
    }
    return true;
}

int main(int argc, char** argv)
{
    HandleCommandLine(argc, argv);

    vr::EVRInitError startupVrErr = vr::VRInitError_None;
    if (!g_desktopPreview)
    {
        vr::VR_Init(&startupVrErr, vr::VRApplication_Background);
        if (startupVrErr != vr::VRInitError_None)
        {
            vr::VR_Shutdown();
            g_desktopPreview = true;
        }
    }

#ifdef _WIN32
    if (!AcquireInstance())
    {
        if (!g_desktopPreview)
            vr::VR_Shutdown();
        return EXIT_SUCCESS;
    }
#endif
    if (g_desktopPreview)
    {
        char note[160];
        snprintf(note, sizeof note, "started (desktop mode, SteamVR not available: %s)",
            g_desktopForced ? "forced by -ui" : vr::VR_GetVRInitErrorAsSymbol(startupVrErr));
        lighthouse::Note(note);
    }
    else
        lighthouse::Note(g_handoverLaunch ? "started (SteamVR mode, handover from desktop instance)" : "started (SteamVR mode)");

    if (!g_desktopPreview)
    {
        try {
            ActivateMultipleDrivers();
        }
        catch (std::exception& ex) {
            lighthouse::Note(ex.what());
        }

        UpdateApplicationRefreshRate();

        try {
            if (!OpenVRManifestInstalled(APP_KEY))
            {
                OpenVRManifestInstall(ExeDirectory());
                lighthouse::Note("manifest registered with SteamVR on first run");
            }
        }
        catch (std::exception& ex) {
            lighthouse::Note(ex.what());
        }

#ifdef _WIN32
        if (!AutoLaunchInitialized() && OpenVRManifestInstalled(APP_KEY))
        {
            vr::EVRApplicationError autoErr = vr::VRApplications()->SetApplicationAutoLaunch(APP_KEY, true);
            if (autoErr == vr::VRApplicationError_None)
            {
                MarkAutoLaunchInitialized();
                lighthouse::Note("autolaunch with SteamVR enabled");
            }
            else
                lighthouse::Note(vr::VRApplications()->GetApplicationsErrorNameFromEnum(autoErr));
        }
#endif
    }

    try {
        if (!g_desktopPreview)
        {
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
    }
    catch (std::exception& ex) {
        lighthouse::Note(ex.what());
    }

    sound::Init();
    lighthouse::Init();
    std::atexit([] { lighthouse::Shutdown(); sound::Shutdown(); });
    SDL_SetHint(SDL_HINT_WINDOW_ACTIVATE_WHEN_SHOWN, "0");
    if (!SDL_Init(SDL_INIT_VIDEO)) {
#ifdef _WIN32
        MessageBoxA(NULL, SDL_GetError(), APP_NAME, MB_OK);
#else
        printf("%s\n\n", SDL_GetError());
#endif
        return EXIT_FAILURE;
    }

    g_skipOpenVRVulkanExtensions = g_desktopPreview;
    try {
        g_vulkanRenderer->Initialize();
        g_imGuiWindow->Initialize(g_vulkanRenderer, g_overlay, APP_NAME, WIN_WIDTH, WIN_HEIGHT);
        g_imGuiWindow->Show();
        SDL_AddEventWatch(&LiveResizeWatch, nullptr);
    } catch (std::exception& ex) {
#ifdef _WIN32
        MessageBoxA(NULL, ex.what(), APP_NAME, MB_OK);
#else
        printf("%s\n\n", ex.what());
#endif
        return EXIT_FAILURE;
    }

    if (!g_desktopPreview)
    {
        InitCalibrator();
        if (!DriverConnected())
            lighthouse::Note("driver not reachable at startup, will keep retrying");
    }
    try {
        LoadProfile(CalCtx);
    }
    catch (std::exception& ex) {
        lighthouse::Note(ex.what());
    }

    lighthouse::SetEnabled(CalCtx.basestationControl);
    if (!g_desktopForced && CalCtx.dynamicBasestationPower)
        lighthouse::SetAutoWake(true);
    lighthouse::EnsureScanning();

    SDL_Event event = {};
    vr::VREvent_t vr_event = {};
    vr::VREvent_t sys_event = {};

    while (g_ticking)
    {
        while (SDL_PollEvent(&event))
        {
            const bool desktopMouseEvent = event.type == SDL_EVENT_MOUSE_MOTION || event.type == SDL_EVENT_MOUSE_BUTTON_DOWN
                || event.type == SDL_EVENT_MOUSE_BUTTON_UP || event.type == SDL_EVENT_MOUSE_WHEEL;
            if (!(g_dashboardActive && desktopMouseEvent))
                ImGui_ImplSDL3_ProcessEvent(&event);

            if (event.type == SDL_EVENT_WINDOW_MINIMIZED && event.window.windowID == SDL_GetWindowID(g_imGuiWindow->Window()))
                g_imGuiWindow->SetMinimizedFromEvent(true);
            if (event.type == SDL_EVENT_WINDOW_RESTORED && event.window.windowID == SDL_GetWindowID(g_imGuiWindow->Window()))
                g_imGuiWindow->SetMinimizedFromEvent(false);
            if (event.type == SDL_EVENT_WINDOW_CLOSE_REQUESTED && event.window.windowID == SDL_GetWindowID(g_imGuiWindow->Window()))
                BeginClose();
            if (event.type == SDL_EVENT_QUIT)
                BeginClose();
        }

#ifdef _WIN32
        if (g_showWindowEvent && WaitForSingleObject(g_showWindowEvent, 0) == WAIT_OBJECT_0 && !g_shutdownScreen)
            g_imGuiWindow->ShowAndFocus();
        if (g_handoverEvent && WaitForSingleObject(g_handoverEvent, 0) == WAIT_OBJECT_0 && !g_shutdownScreen)
        {
            lighthouse::Note("handover requested by the SteamVR instance, closing desktop instance");
            MarkExiting();
            g_handoverToVrInstance = true;
            g_ticking = false;
        }
#endif

        g_imGuiWindow->SetClosing(g_shutdownScreen);
        if (g_shutdownScreen)
        {
            if (lighthouse::Idle())
            {
                lighthouse::Note("exit: standby screen finished, closing");
                g_exitStandbyDone = true;
                g_ticking = false;
            }
            else if (SDL_GetTicks() - g_shutdownStart > g_shutdownBudgetMs)
            {
                lighthouse::Note("exit: standby screen timed out, closing anyway");
                g_ticking = false;
            }
        }

#ifdef _WIN32
        if (g_ticking && g_desktopPreview && !g_desktopForced && !g_shutdownScreen)
        {
            static uint64_t lastVrProbe = 0;
            uint64_t nowMs = SDL_GetTicks();
            if (nowMs - lastVrProbe > 5000)
            {
                lastVrProbe = nowMs;
                bool ready = false;
                if (SteamVRProcessRunning())
                {
                    vr::EVRInitError probeErr = vr::VRInitError_None;
                    vr::VR_Init(&probeErr, vr::VRApplication_Background);
                    vr::VR_Shutdown();
                    ready = probeErr == vr::VRInitError_None;
                }
                if (ready)
                {
                    char exe[MAX_PATH] = {};
                    GetModuleFileNameA(nullptr, exe, MAX_PATH);
                    std::string dir = ExeDirectory();
                    char cmd[MAX_PATH + 32];
                    snprintf(cmd, sizeof cmd, "\"%s\" -handover", exe);
                    MarkExiting();
                    STARTUPINFOA si = { sizeof(si) };
                    PROCESS_INFORMATION pi = {};
                    if (CreateProcessA(NULL, cmd, NULL, NULL, FALSE, 0, NULL, dir.c_str(), &si, &pi))
                    {
                        CloseHandle(pi.hProcess);
                        CloseHandle(pi.hThread);
                        lighthouse::Note("steamvr detected, handing over to a full-mode instance");
                        g_handoverToVrInstance = true;
                        g_ticking = false;
                    }
                    else
                        lighthouse::Note("steamvr detected, but relaunch failed; staying in desktop mode");
                }
            }
        }
#endif

        while (!g_desktopPreview && vr::VRSystem() && vr::VRSystem()->PollNextEvent(&sys_event, sizeof(sys_event)))
        {
            if (sys_event.eventType == vr::VREvent_Quit)
            {
                lighthouse::Note("steamvr is shutting down");
                vr::VRSystem()->AcknowledgeQuit_Exiting();
                ImGui_ImplOpenVR_Detach();
                vr::VR_Shutdown();
                g_desktopPreview = true;
                g_dashboardActive = false;
                BeginClose();
                break;
            }
        }

        while (!g_desktopPreview && g_overlay->Handle() != vr::k_ulOverlayHandleInvalid && vr::VROverlay()->PollNextOverlayEvent(g_overlay->Handle(), &vr_event, sizeof(vr_event)))
        {
            ImGui_ImplOpenVR_ProcessOverlayEvent(vr_event);

            static int loggedClicks = 0;
            if (vr_event.eventType == vr::VREvent_MouseButtonDown && loggedClicks < 30)
            {
                loggedClicks++;
                char note[160];
                snprintf(note, sizeof note, "overlay click at (%.0f, %.0f), button %u, ui size %.0fx%.0f",
                    vr_event.data.mouse.x, vr_event.data.mouse.y, vr_event.data.mouse.button,
                    ImGui::GetIO().DisplaySize.x, ImGui::GetIO().DisplaySize.y);
                lighthouse::Note(note);
            }

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
                {
                    lighthouse::Note("steamvr is shutting down");
                    vr::VRSystem()->AcknowledgeQuit_Exiting();
                    ImGui_ImplOpenVR_Detach();
                    vr::VR_Shutdown();
                    g_desktopPreview = true;
                    g_dashboardActive = false;
                    BeginClose();
                    break;
                }
            }
        }

        const double time = static_cast<double>(SDL_GetTicks()) / 1000.0;
        if (!g_desktopPreview)
        {
            try {
                CalibrationTick(time);
            }
            catch (std::exception& ex) {
                lighthouse::Note(ex.what());
            }
        }

        RenderFrame();

        static int loggedUiClicks = 0;
        if (g_dashboardActive && ImGui::GetIO().MouseClicked[0] && loggedUiClicks < 30)
        {
            loggedUiClicks++;
            char note[128];
            snprintf(note, sizeof note, "ui registered click at (%.0f, %.0f), hovering ui: %d",
                ImGui::GetIO().MousePos.x, ImGui::GetIO().MousePos.y, ImGui::GetIO().WantCaptureMouse ? 1 : 0);
            lighthouse::Note(note);
        }

        const uint64_t target_time_ns = static_cast<uint64_t>(1'000'000'000.0 / g_hmd_refresh_rate);
        uint64_t elapsed_ns = SDL_GetTicksNS() - g_last_frame_time;

        if (elapsed_ns < target_time_ns)
        {
            if (!g_desktopPreview)
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

    lighthouse::Note("main loop exited");
#ifdef _WIN32
    MarkExiting();
#endif
    if (!g_exitStandbyDone && !g_desktopForced && !g_handoverToVrInstance && CalCtx.dynamicBasestationPower)
        lighthouse::StandbyAllAndWait(8000);
    lighthouse::Note("teardown");
    lighthouse::Shutdown();
    sound::Shutdown();
    SDL_Quit();
    if (!g_desktopPreview)
        vr::VR_Shutdown();
#ifdef _WIN32
    ReleaseInstance();
#endif

    return 0;
}

static std::string SteamVRConfigDir()
{
    const char* localAppData = getenv("LOCALAPPDATA");
    if (!localAppData)
        return {};
    std::ifstream in(std::string(localAppData) + "\\openvr\\openvrpaths.vrpath");
    if (!in)
        return {};
    std::stringstream buf;
    buf << in.rdbuf();
    picojson::value root;
    if (!picojson::parse(root, buf.str()).empty() || !root.is<picojson::object>())
        return {};
    const auto& obj = root.get<picojson::object>();
    auto it = obj.find("config");
    if (it == obj.end() || !it->second.is<picojson::array>() || it->second.get<picojson::array>().empty())
        return {};
    const auto& first = it->second.get<picojson::array>()[0];
    return first.is<std::string>() ? first.get<std::string>() : std::string();
}

static bool EditJsonFile(const std::string& path, const std::function<void(picojson::object&)>& edit)
{
    picojson::value root = picojson::value(picojson::object());
    {
        std::ifstream in(path);
        if (in)
        {
            std::stringstream buf;
            buf << in.rdbuf();
            std::string text = buf.str();
            if (!text.empty())
            {
                picojson::value parsed;
                if (!picojson::parse(parsed, text).empty() || !parsed.is<picojson::object>())
                    return false;
                root = parsed;
            }
        }
    }
    edit(root.get<picojson::object>());
    std::ofstream out(path, std::ios::trunc);
    if (!out)
        return false;
    out << root.serialize(true);
    return (bool)out;
}

static std::string LowerCase(std::string s)
{
    std::transform(s.begin(), s.end(), s.begin(), [](unsigned char c) { return (char)tolower(c); });
    return s;
}

static bool IsSpaceSyncManifestPath(const std::string& path)
{
    std::string lower = LowerCase(path);
    const std::string tail = "spacesync\\manifest.vrmanifest";
    return lower.size() >= tail.size() && lower.compare(lower.size() - tail.size(), tail.size(), tail) == 0;
}

static int WriteManifestRegistration(bool install)
{
    std::string config = SteamVRConfigDir();
    if (config.empty())
    {
        fprintf(stderr, "SteamVR config directory not found\n");
        return -2;
    }

    std::string manifest = ExeDirectory() + "\\manifest.vrmanifest";
    bool ok = EditJsonFile(config + "\\appconfig.json", [&](picojson::object& obj) {
        picojson::array paths;
        auto it = obj.find("manifest_paths");
        if (it != obj.end() && it->second.is<picojson::array>())
            for (const auto& v : it->second.get<picojson::array>())
                if (!v.is<std::string>() || !IsSpaceSyncManifestPath(v.get<std::string>()))
                    paths.push_back(v);
        if (install)
            paths.push_back(picojson::value(manifest));
        obj["manifest_paths"] = picojson::value(paths);
    });
    if (!ok)
    {
        fprintf(stderr, "Could not update appconfig.json\n");
        return -3;
    }

    if (install)
    {
        std::string dir = config + "\\vrappconfig";
        CreateDirectoryA(dir.c_str(), NULL);
        ok = EditJsonFile(dir + "\\" + APP_KEY + ".vrappconfig", [](picojson::object& obj) {
            obj["autolaunch"] = picojson::value(true);
            if (obj.find("last_launch_time") == obj.end())
                obj["last_launch_time"] = picojson::value(std::string("0"));
        });
        if (!ok)
        {
            fprintf(stderr, "Could not enable autolaunch\n");
            return -4;
        }
    }
    return 0;
}

static int WriteActivateMultipleDrivers()
{
    std::string config = SteamVRConfigDir();
    if (config.empty())
    {
        fprintf(stderr, "SteamVR config directory not found\n");
        return -2;
    }
    bool ok = EditJsonFile(config + "\\steamvr.vrsettings", [](picojson::object& obj) {
        picojson::object section;
        auto it = obj.find("steamvr");
        if (it != obj.end() && it->second.is<picojson::object>())
            section = it->second.get<picojson::object>();
        section["activateMultipleDrivers"] = picojson::value(true);
        obj["steamvr"] = picojson::value(section);
    });
    if (!ok)
    {
        fprintf(stderr, "Could not update steamvr.vrsettings\n");
        return -3;
    }
    return 0;
}

static auto HandleCommandLine(int argc, char** argv) -> void
{
    if (argc < 2)
        return;

    const std::string arg = argv[1];

    if (arg == "-ui")
    {
        g_desktopPreview = true;
        g_desktopForced = true;
        return;
    }

    if (arg == "-handover")
    {
        g_handoverLaunch = true;
        return;
    }

    char cwd[1024] = { 0 };
    _getcwd(cwd, sizeof(cwd));

    if (arg == "-openvrpath")
    {
        char runtimePath[1024] = { 0 };
        unsigned int pathLen = 0;
        if (vr::VR_GetRuntimePath(runtimePath, sizeof(runtimePath), &pathLen) && runtimePath[0])
        {
            printf("%s", runtimePath);
            fflush(stdout);
            exit(0);
        }
        fprintf(stderr, "SteamVR runtime path not found\n");
        exit(-2);
    }
    else if (arg == "-installmanifest")
    {
        exit(WriteManifestRegistration(true));
    }
    else if (arg == "-removemanifest")
    {
        exit(WriteManifestRegistration(false));
    }
    else if (arg == "-activatemultipledrivers")
    {
        exit(WriteActivateMultipleDrivers());
    }
}
