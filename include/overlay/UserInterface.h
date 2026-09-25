// SPDX-License-Identifier: AGPL-3.0-only
// Modified by Shinyflvres, 2026-08-23. Part of SpaceSync, a modified version of OpenVR-SpaceOverride by Nyabsi (AGPL-3.0). See NOTICE.md

#pragma once

#include <string>
#include <vector>

#include <openvr.h>

#include "TrackingPreview.h"

struct VRDevice
{
	int id = -1;
	vr::TrackedDeviceClass deviceClass;
	std::string model = "";
	std::string serial = "";
	std::string trackingSystem = "";
	vr::ETrackedControllerRole controllerRole = vr::TrackedControllerRole_Invalid;
};

struct VRState
{
	std::vector<std::string> trackingSystems;
	std::vector<VRDevice> devices;
};

class UserInterface
{
public:
	enum class WindowAction { None, Minimize, Close };
	static constexpr float DesignWidth = 1080.0f;
	static constexpr float DesignHeight = 700.0f;
	static constexpr float TitleBarHeight = 40.0f;
	static constexpr float TitleBarButtonWidth = 40.0f;
	static constexpr int TitleBarButtonCount = 2;

	WindowAction Render(bool runningInOverlay);

private:
	enum class Tab { Calibration, Preview, Smoothing, Lighthouse, Settings };

	struct Status
	{
		const VRDevice* hmd = nullptr;
		const VRDevice* tracker = nullptr;
		std::string headline; 
		std::string detail;    
		unsigned color = 0;     
		bool ok = false;
	};

	void CollectDevices(VRState& state) const;
	Status BuildStatus(const VRState& state) const;

	WindowAction RenderTitleBar();
	void RenderTabs();
	void RenderCalibration(const Status& status);
	void RenderPreview();
	void RenderLighthouse();
	void RenderEdit(const Status& status);
	void RenderSmoothing();
	void RenderSettings();
	void RenderFooter();
	void RenderWizard();
	void RenderConfirm();

	Tab tab_ = Tab::Calibration;
	TrackingPreview preview_;
	bool smoothingDirty_ = false;
	bool editView_ = false;
	double editStep_ = 0.1;
	bool confirmRemove_ = false;
	bool wizardOpen_ = false;
	float scrollTarget_ = 0.0f;
	float scrollApplied_ = 0.0f;
	int scrollTab_ = -1;
	float contentTop_ = 0.0f;
	float contentHeight_ = 0.0f;
};
