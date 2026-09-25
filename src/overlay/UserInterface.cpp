// SPDX-License-Identifier: AGPL-3.0-only
// Modified by Shinyflvres, 2026-08-23. Part of SpaceSync, a modified version of OpenVR-SpaceOverride by Nyabsi (AGPL-3.0). See NOTICE.md

#include "UserInterface.h"
#include "Calibration.h"
#include "Configuration.h"
#include "Lighthouse.h"
#include "Theme.h"
#include "Version.h"

#include <string>
#include <vector>
#include <algorithm>
#include <cstdio>
#include <cmath>
#include <imgui.h>

using namespace ui;

static const char* kCreditLine = "SpaceSync Beta by Shinyflvres. A modified version of OpenVR-SpaceOverride by Nyabsi (AGPL-3.0). Thanks to tach/pushrax for SpaceCalibrator.";

namespace
{
	struct WizardStep { const char* title; Icon icon; };
	const WizardStep kSteps[] = {
		{ "Look left", Icon::ArrowLeft },
		{ "Look straight ahead", Icon::Dot },
		{ "Look right", Icon::ArrowRight },
		{ "Look straight ahead", Icon::Dot },
		{ "Look up", Icon::ArrowUp },
		{ "Look straight ahead", Icon::Dot },
		{ "Look down", Icon::ArrowDown },
		{ "Look straight ahead", Icon::Dot },
	};
	constexpr int kStepCount = (int)(sizeof(kSteps) / sizeof(kSteps[0]));

	// Last progress + last log line from the calibration.
	void CalibrationFeedback(float& fraction, std::string& lastLine)
	{
		fraction = 0.0f;
		lastLine.clear();
		for (auto it = CalCtx.messages.rbegin(); it != CalCtx.messages.rend(); ++it)
		{
			if (it->type == CalibrationContext::Message::Progress)
			{
				if (it->target > 0)
					fraction = (float)it->progress / (float)it->target;
				break;
			}
		}
		for (auto it = CalCtx.messages.rbegin(); it != CalCtx.messages.rend() && lastLine.empty(); ++it)
		{
			if (it->type != CalibrationContext::Message::String)
				continue;
			const std::string& s = it->str;
			size_t end = s.find_last_not_of("\r\n");
			if (end == std::string::npos)
				continue;
			size_t start = s.find_last_of("\r\n", end);
			lastLine = s.substr(start == std::string::npos ? 0 : start + 1, end - (start == std::string::npos ? 0 : start + 1) + 1);
		}
	}

	// Text segments on one line with mixed fonts/colours.
	struct Segment { ImFont* font; float size; unsigned color; std::string text; };
	void InlineText(const std::vector<Segment>& segments)
	{
		ImDrawList* dl = ImGui::GetWindowDrawList();
		ImVec2 p = ImGui::GetCursorScreenPos();
		float x = p.x;
		float maxH = 0.0f;
		std::vector<ImVec2> sizes;
		for (auto& s : segments)
		{
			ImVec2 sz = TextSize(s.font, s.size, s.text.c_str());
			sizes.push_back(sz);
			maxH = std::max(maxH, sz.y);
		}
		for (size_t i = 0; i < segments.size(); i++)
		{
			const Segment& s = segments[i];
			DrawText(dl, s.font, s.size, ImVec2(x, p.y + (maxH - sizes[i].y) * 0.5f), s.color, s.text.c_str());
			x += sizes[i].x;
		}
		ImGui::Dummy(ImVec2(x - p.x, maxH));
	}
}

// -----------------------------------------------------------------------------------------

void UserInterface::CollectDevices(VRState& state) const
{
	auto& trackingSystems = state.trackingSystems;
	char buffer[vr::k_unMaxPropertyStringSize];

	for (uint32_t id = 0; id < vr::k_unMaxTrackedDeviceCount; ++id)
	{
		vr::ETrackedPropertyError err = vr::TrackedProp_Success;
		auto deviceClass = vr::VRSystem()->GetTrackedDeviceClass(id);
		if (deviceClass == vr::TrackedDeviceClass_Invalid || deviceClass == vr::TrackedDeviceClass_TrackingReference)
			continue;

		vr::VRSystem()->GetStringTrackedDeviceProperty(id, vr::Prop_TrackingSystemName_String, buffer, vr::k_unMaxPropertyStringSize, &err);
		if (err != vr::TrackedProp_Success)
			continue;

		std::string system(buffer);
		auto existing = std::find(trackingSystems.begin(), trackingSystems.end(), system);
		if (existing != trackingSystems.end())
		{
			if (deviceClass == vr::TrackedDeviceClass_HMD)
			{
				trackingSystems.erase(existing);
				trackingSystems.insert(trackingSystems.begin(), system);
			}
		}
		else
		{
			trackingSystems.push_back(system);
		}

		VRDevice device;
		device.id = id;
		device.deviceClass = deviceClass;
		device.trackingSystem = system;

		vr::VRSystem()->GetStringTrackedDeviceProperty(id, vr::Prop_ModelNumber_String, buffer, vr::k_unMaxPropertyStringSize, &err);
		device.model = std::string(buffer);

		vr::VRSystem()->GetStringTrackedDeviceProperty(id, vr::Prop_SerialNumber_String, buffer, vr::k_unMaxPropertyStringSize, &err);
		device.serial = std::string(buffer);

		device.controllerRole = (vr::ETrackedControllerRole)vr::VRSystem()->GetInt32TrackedDeviceProperty(id, vr::Prop_ControllerRoleHint_Int32, &err);
		state.devices.push_back(device);
	}
}

UserInterface::Status UserInterface::BuildStatus(const VRState& state) const
{
	Status s;
	for (auto& device : state.devices)
	{
		if (device.id == vr::k_unTrackedDeviceIndex_Hmd)
			s.hmd = &device;
		if (!CalCtx.trackerSerial.empty() && device.serial == CalCtx.trackerSerial)
			s.tracker = &device;
	}

	if (!CalCtx.validProfile)
	{
		s.headline = "No calibration yet";
		s.detail = "Click the circle to calibrate";
		s.color = P.textMuted;
	}
	else if (!s.tracker && !CalCtx.noHeadTracker)
	{
		s.headline = "Headset tracker not connected";
		s.detail = CalCtx.trackerSerial;
		s.color = P.danger;
	}
	else if (CalCtx.noHeadTracker)
	{
		s.headline = "HMD Driven, one-time calibration";
		s.detail = "no running alignment";
		s.color = P.green;
		s.ok = true;
	}
	else if (!CalCtx.enabled)
	{
		s.headline = "Override disabled";
		s.detail = "HMD tracking system changed?";
		s.color = P.danger;
	}
	else if (CalCtx.followSlamHmd)
	{
		s.headline = "HMD Driven active";
		s.detail = s.tracker->serial;
		s.color = P.green;
		s.ok = true;
	}
	else
	{
		s.headline = "Lighthouse Driven active";
		s.detail = s.tracker->serial;
		s.color = P.green;
		s.ok = true;
	}
	return s;
}

// -----------------------------------------------------------------------------------------

// Content width in design px (minus the 24px padding).
static float AvailDesignWidth()
{
	return std::max(200.0f, (ImGui::GetIO().DisplaySize.x - 2.0f * px(24.0f)) / S());
}

static const float PageWidth = 900.0f;

static float PageInset()
{
	return std::max(0.0f, (AvailDesignWidth() - PageWidth) * 0.5f);
}

static float PageDesignWidth()
{
	return std::min(PageWidth, AvailDesignWidth());
}

UserInterface::WindowAction UserInterface::Render(bool runningInOverlay)
{
	ui::SetContentScale(CalCtx.uiScale);

	ImGuiIO& io = ImGui::GetIO();
	const float W = io.DisplaySize.x;
	const float H = io.DisplaySize.y;

	VRState state;
	CollectDevices(state);
	Status status = BuildStatus(state);

	ImGui::SetNextWindowPos(ImVec2(0.0f, 0.0f));
	ImGui::SetNextWindowSize(io.DisplaySize);
	ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(0.0f, 0.0f));
	ImGui::PushStyleVar(ImGuiStyleVar_WindowBorderSize, 0.0f);
	ImGui::PushStyleVar(ImGuiStyleVar_WindowRounding, 0.0f);

	const ImGuiWindowFlags flags = ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove |
		ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse | ImGuiWindowFlags_NoBringToFrontOnFocus |
		ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoSavedSettings;

	WindowAction action = WindowAction::None;

	if (ImGui::Begin("SpaceSync", nullptr, flags))
	{
		ImDrawList* dl = ImGui::GetWindowDrawList();
		dl->AddRectFilled(ImVec2(0, 0), ImVec2(W, H), Col(P.card));

		float y = 0.0f;
		if (!runningInOverlay)
		{
			action = RenderTitleBar();
			y = px(TitleBarHeight);
		}

		ImGui::SetCursorPos(ImVec2(0.0f, y));
		RenderTabs();
		y = ImGui::GetCursorPosY();

		const float footerH = px(9.0f) + TextSize(F.regular, 11.5f, "X").y + px(9.0f);
		contentTop_ = y;
		contentHeight_ = H - footerH - y;

		ImGui::SetCursorPos(ImVec2(0.0f, y));
		ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(px(24.0f), px(24.0f)));
		if (ImGui::BeginChild("content", ImVec2(W, contentHeight_), ImGuiChildFlags_AlwaysUseWindowPadding, ImGuiWindowFlags_NoScrollWithMouse))
		{
			const float current = ImGui::GetScrollY();

			if (scrollTab_ != (int)tab_ || std::abs(current - scrollApplied_) > 0.5f)
			{
				scrollTab_ = (int)tab_;
				scrollTarget_ = current;
			}

			const float maxScroll = ImGui::GetScrollMaxY();
			if (ImGui::IsWindowHovered(ImGuiHoveredFlags_ChildWindows) && io.MouseWheel != 0.0f)
				scrollTarget_ -= io.MouseWheel * px(110.0f);
			scrollTarget_ = std::min(maxScroll, std::max(0.0f, scrollTarget_));
			const float blend = 1.0f - std::exp(-io.DeltaTime / 0.055f);
			const float next = current + (scrollTarget_ - current) * blend;
			scrollApplied_ = std::abs(scrollTarget_ - next) < 0.5f ? scrollTarget_ : next;
			ImGui::SetScrollY(scrollApplied_);

			const float inset = px(PageInset());
			if (inset > 0.0f)
				ImGui::Indent(inset);

			switch (tab_)
			{
			case Tab::Calibration:
				if (editView_) RenderEdit(status); else RenderCalibration(status);
				break;
			case Tab::Preview:
				RenderPreview();
				break;
			case Tab::Smoothing:
				RenderSmoothing();
				break;
			case Tab::Lighthouse:
				RenderLighthouse();
				break;
			case Tab::Settings:
				RenderSettings();
				break;
			}

			if (inset > 0.0f)
				ImGui::Unindent(inset);
		}
		ImGui::EndChild();
		ImGui::PopStyleVar();

		ImGui::SetCursorPos(ImVec2(0.0f, H - footerH));
		RenderFooter();

		// outer border
		dl->AddRect(ImVec2(0.5f, 0.5f), ImVec2(W - 0.5f, H - 0.5f), Col(P.borderStrong));

		RenderWizard();
		RenderConfirm();
	}
	ImGui::End();
	ImGui::PopStyleVar(3);

	return action;
}

// -----------------------------------------------------------------------------------------

UserInterface::WindowAction UserInterface::RenderTitleBar()
{
	const float W = ImGui::GetIO().DisplaySize.x;
	const float h = px(TitleBarHeight);
	ImDrawList* dl = ImGui::GetWindowDrawList();

	dl->AddRectFilled(ImVec2(0, 0), ImVec2(W, h), Col(P.titleBar));
	dl->AddRectFilled(ImVec2(0, h - 1.0f), ImVec2(W, h), Col(P.border));

	ImVec2 ts = TextSize(F.semibold, 12.5f, "SpaceSync");
	DrawText(dl, F.semibold, 12.5f, ImVec2(px(14.0f), (h - ts.y) * 0.5f), P.textTitle, "SpaceSync");

	WindowAction action = WindowAction::None;
	const float bw = px(TitleBarButtonWidth), bh = px(32.0f);
	float x = W - px(4.0f) - bw * TitleBarButtonCount;
	const float by = (h - bh) * 0.5f;

	struct Btn { const char* id; Icon icon; float size; unsigned hoverBg; unsigned hoverFg; WindowAction action; };
	const Btn buttons[TitleBarButtonCount] = {
		{ "##min", Icon::Minus, 12.0f, P.button, P.textStrong, WindowAction::Minimize },
		{ "##close", Icon::Cross, 10.0f, P.danger, 0xffffff, WindowAction::Close },
	};

	for (const Btn& b : buttons)
	{
		ImGui::SetCursorPos(ImVec2(x, by));
		ImGui::InvisibleButton(b.id, ImVec2(bw, bh));
		bool hovered = ImGui::IsItemHovered();
		if (ImGui::IsItemClicked())
			action = b.action;
		if (hovered)
			dl->AddRectFilled(ImVec2(x, by), ImVec2(x + bw, by + bh), Col(b.hoverBg));
		DrawIcon(dl, b.icon, ImVec2(x + bw * 0.5f, by + bh * 0.5f), b.size, Col(hovered ? b.hoverFg : P.textIcon), 1.3f);
		x += bw;
	}
	return action;
}

void UserInterface::RenderTabs()
{
	const float W = ImGui::GetIO().DisplaySize.x;
	const float y0 = ImGui::GetCursorPosY();
	const float h = TextSize(F.semibold, 13.0f, "X").y + px(22.0f);
	ImDrawList* dl = ImGui::GetWindowDrawList();
	ImVec2 wp = ImGui::GetWindowPos();

	dl->AddRectFilled(ImVec2(wp.x, wp.y + y0 + h - 1.0f), ImVec2(wp.x + W, wp.y + y0 + h), Col(P.border));

	ImGui::SetCursorPos(ImVec2(px(24.0f + PageInset()), y0));
	if (TabItem("Calibration", tab_ == Tab::Calibration)) tab_ = Tab::Calibration;
	ImGui::SameLine();
	if (TabItem("Tracking Preview", tab_ == Tab::Preview)) tab_ = Tab::Preview;
	ImGui::SameLine();
	if (TabItem("Smoothing", tab_ == Tab::Smoothing)) tab_ = Tab::Smoothing;
	ImGui::SameLine();
	if (TabItem("Lighthouse", tab_ == Tab::Lighthouse)) tab_ = Tab::Lighthouse;
	ImGui::SameLine();
	if (TabItem("Settings", tab_ == Tab::Settings)) tab_ = Tab::Settings;

	ImGui::SetCursorPos(ImVec2(0.0f, y0 + h));
	ImGui::Dummy(ImVec2(0.0f, 0.0f));
}

void UserInterface::RenderFooter()
{
	const float W = ImGui::GetIO().DisplaySize.x;
	const float y0 = ImGui::GetCursorPosY();
	ImVec2 wp = ImGui::GetWindowPos();
	const float H = ImGui::GetIO().DisplaySize.y;
	ImDrawList* dl = ImGui::GetWindowDrawList();

	dl->AddRectFilled(ImVec2(wp.x, wp.y + y0), ImVec2(wp.x + W, wp.y + H), Col(P.titleBar));
	dl->AddRectFilled(ImVec2(wp.x, wp.y + y0), ImVec2(wp.x + W, wp.y + y0 + 1.0f), Col(P.border));
	DrawText(dl, F.regular, 11.5f, ImVec2(wp.x + px(24.0f + PageInset()), wp.y + y0 + px(9.0f)), P.textFooter, kCreditLine);
	ImGui::Dummy(ImVec2(W, H - y0));
}

// -----------------------------------------------------------------------------------------

void UserInterface::RenderCalibration(const Status& status)
{
	ImDrawList* dl = ImGui::GetWindowDrawList();
	const bool calibrated = CalCtx.validProfile;
	const float W = ImGui::GetIO().DisplaySize.x;
	// 400px circle, shrunk if the content area is smaller.
	const float buttonH = TextSize(F.medium, 13.0f, "X").y + px(18.0f);
	const float maxD = contentHeight_ - 2.0f * px(24.0f) - px(16.0f + 28.0f + 16.0f) - buttonH;
	const float d = std::max(px(260.0f), std::min(px(400.0f), maxD));

	// Big circle
	ImVec2 origin = ImGui::GetCursorScreenPos();
	ImVec2 c(ImGui::GetWindowPos().x + W * 0.5f, origin.y + px(16.0f) + d * 0.5f);

	ImGui::SetCursorScreenPos(ImVec2(c.x - d * 0.5f, c.y - d * 0.5f));
	ImGui::InvisibleButton("##circle", ImVec2(d, d));
	bool hovered = false;
	if (ImGui::IsItemHovered())
	{
		ImVec2 m = ImGui::GetIO().MousePos;
		float dx = m.x - c.x, dy = m.y - c.y;
		hovered = dx * dx + dy * dy <= (d * 0.5f) * (d * 0.5f);
		if (hovered)
			ImGui::SetMouseCursor(ImGuiMouseCursor_Hand);
	}
	if (hovered && ImGui::IsItemClicked() && CalCtx.state == CalibrationState::None && !wizardOpen_)
	{
		StartCalibration();
		wizardOpen_ = true;
	}

	dl->AddCircleFilled(c, d * 0.5f, Col(hovered ? 0x14181e : P.inputBg), 96);
	dl->AddCircle(c, d * 0.5f, Col(calibrated ? P.greenBorder : P.blueBorder), 96, 1.0f);

	// Text stack inside the circle
	const float lineH = px(18.0f);  // 12px text with line-height 1.5
	const float stackH = px(28 + 14 + 19 + 6 + 12.5f + 20 + 1 + 16) + 3.0f * lineH + px(12.0f) + 2.0f * lineH;
	float y = c.y - stackH * 0.5f;

	DrawIcon(dl, calibrated ? Icon::Check : Icon::Ring, ImVec2(c.x, y + px(14.0f)), 26.0f, Col(calibrated ? P.green : P.link), 2.2f);
	y += px(28.0f + 14.0f);
	DrawTextCentered(dl, F.semibold, 19.0f, ImVec2(c.x, y + px(9.5f)), P.textBright, calibrated ? "Calibrated" : "Calibrate");
	y += px(19.0f + 6.0f);
	DrawTextCentered(dl, F.regular, 12.5f, ImVec2(c.x, y + px(6.25f)), P.textMuted, calibrated ? "Click to calibrate again" : "Click to start");
	y += px(12.5f + 20.0f);
	dl->AddRectFilled(ImVec2(c.x - px(75.0f), y), ImVec2(c.x + px(75.0f), y + 1.0f), Col(P.border));
	y += 1.0f + px(16.0f);

	DrawTextCentered(dl, F.regular, 12.0f, ImVec2(c.x, y + lineH * 0.5f), P.textMuted, "HMD");
	y += lineH;
	DrawTextCentered(dl, F.mono, 12.0f, ImVec2(c.x, y + lineH * 0.5f), P.text, status.hmd ? status.hmd->serial.c_str() : "not detected");
	y += lineH;
	DrawTextCentered(dl, F.regular, 12.0f, ImVec2(c.x, y + lineH * 0.5f), P.textMuted, status.hmd ? status.hmd->trackingSystem.c_str() : "");
	y += lineH + px(12.0f);

	DrawTextCentered(dl, F.regular, 12.0f, ImVec2(c.x, y + lineH * 0.5f), status.color, status.headline.c_str());
	y += lineH;
	DrawTextCentered(dl, F.mono, 12.0f, ImVec2(c.x, y + lineH * 0.5f), status.color, status.detail.c_str());

	// Buttons below the circle
	ImGui::SetCursorScreenPos(ImVec2(c.x, c.y + d * 0.5f + px(28.0f)));
	ButtonOpts edit;
	edit.enabled = calibrated;
	ButtonOpts remove;
	remove.enabled = calibrated;
	float editW = TextSize(F.medium, 13.0f, "Edit Calibration").x + px(36.0f);
	float removeW = TextSize(F.medium, 13.0f, "Remove Calibration").x + px(36.0f);
	float total = editW + px(8.0f) + removeW;
	ImGui::SetCursorScreenPos(ImVec2(c.x - total * 0.5f, ImGui::GetCursorScreenPos().y));
	if (Button("Edit Calibration", edit))
	{
		editView_ = true;
		CalCtx.state = CalibrationState::Editing;
	}
	ImGui::SameLine(0.0f, px(8.0f));
	if (Button("Remove Calibration", remove))
		confirmRemove_ = true;

	if (calibrated && CalCtx.driverStatus.mountShiftSuspected)
	{
		VSpace(18.0f);
		char warn[192];
		std::snprintf(warn, sizeof warn, "The head tracker seems to have moved on the headset. The driver corrected %.1f deg / %.1f cm so far, a fresh calibration is the clean fix.",
			CalCtx.driverStatus.tiltDeg, CalCtx.driverStatus.translationDeviationM * 100.0);
		ImVec2 ws = TextSize(F.regular, 13.0f, warn);
		ImGui::SetCursorScreenPos(ImVec2(c.x - ws.x * 0.5f, ImGui::GetCursorScreenPos().y));
		Text(F.regular, 13.0f, P.yellow, warn);
	}

	VSpace(16.0f);
}

void UserInterface::RenderEdit(const Status& status)
{
	// Header lines
	{
		std::vector<Segment> line1 = {
			{ F.regular, 13.0f, P.textMuted, "HMD: " },
			{ F.mono, 12.5f, P.text, status.hmd ? status.hmd->serial : std::string("not detected") },
			{ F.regular, 13.0f, P.textMuted, status.hmd ? " (" + status.hmd->trackingSystem + ")" : std::string() },
		};
		InlineText(line1);
		VSpace(5.0f);
		std::vector<Segment> line2 = {
			{ F.regular, 13.0f, status.color, status.headline + (status.detail.empty() ? "" : " via ") },
			{ F.mono, 12.5f, status.color, status.detail },
		};
		InlineText(line2);
		VSpace(22.0f);
	}

	const float maxW = std::min(760.0f, PageDesignWidth());
	const float x0 = ImGui::GetCursorPosX();

	// Back link + step size pills
	{
		float y0 = ImGui::GetCursorPosY();
		if (Link("Back"))
		{
			LoadProfile(CalCtx);
			CalCtx.state = CalibrationState::None;
			editView_ = false;
			return;
		}
		float rowH = ImGui::GetItemRectSize().y;

		const char* labels[3] = { "0.01", "0.1", "1.0" };
		const double steps[3] = { 0.01, 0.1, 1.0 };
		float pillW[3], totalW = 0.0f;
		for (int i = 0; i < 3; i++)
		{
			pillW[i] = TextSize(F.mono, 12.0f, labels[i]).x + px(20.0f);
			totalW += pillW[i] + (i ? px(6.0f) : 0.0f);
		}
		float x = x0 + px(maxW) - totalW;
		float pillH = TextSize(F.mono, 12.0f, "0").y + px(10.0f);
		for (int i = 0; i < 3; i++)
		{
			ImGui::SetCursorPos(ImVec2(x, y0 + (rowH - pillH) * 0.5f));
			if (Pill(labels[i], editStep_ == steps[i]))
				editStep_ = steps[i];
			x += pillW[i] + px(6.0f);
		}
		ImGui::SetCursorPos(ImVec2(x0, y0 + std::max(rowH, pillH)));
		VSpace(18.0f);
	}

	struct Field { const char* label; double* value; double stepMul; bool worksInHmdDriven; };
	Field rows[3][3] = {
		{ { "Yaw", &CalCtx.calibratedRotation(1), 1.0, false }, { "Pitch", &CalCtx.calibratedRotation(2), 1.0, true }, { "Roll", &CalCtx.calibratedRotation(0), 1.0, true } },
		{ { "X", &CalCtx.calibratedTranslation(0), 1.0, false }, { "Y", &CalCtx.calibratedTranslation(1), 1.0, false }, { "Z", &CalCtx.calibratedTranslation(2), 1.0, false } },
		{ { "Scale", &CalCtx.calibratedScale, 0.01, false }, { "HMD Scale", &CalCtx.hmdScale, 0.01, true }, { nullptr, nullptr, 0.0, false } },
	};

	if (CalCtx.followSlamHmd)
	{
		TextWrapped(F.regular, 12.0f, P.textDim, maxW,
			"Greyed out fields do nothing in HMD Driven mode: the runtime alignment measures yaw and position against your head every frame and undoes those edits within a few seconds. Switch to Lighthouse Driven to use them.");
		VSpace(14.0f);
	}

	const float colW = (maxW - 2.0f * 14.0f) / 3.0f;
	for (int r = 0; r < 3; r++)
	{
		float yRow = ImGui::GetCursorPosY();
		float labelH = TextSize(F.regular, 12.5f, "X").y;
		for (int col = 0; col < 3; col++)
		{
			const Field& f = rows[r][col];
			if (!f.label)
				continue;
			const bool live = f.worksInHmdDriven || !CalCtx.followSlamHmd;
			float x = x0 + col * px(colW + 14.0f);
			ImGui::SetCursorPos(ImVec2(x, yRow));
			Text(F.regular, 12.5f, live ? P.textMuted : P.textDisabled, f.label);
			ImGui::SetCursorPos(ImVec2(x, yRow + labelH + px(6.0f)));
			char id[32];
			std::snprintf(id, sizeof id, "##field%d%d", r, col);
			Stepper(id, f.value, editStep_ * f.stepMul, 8, colW, live);
		}
		ImGui::SetCursorPos(ImVec2(x0, yRow + labelH + px(6.0f + 34.0f)));
		ImGui::Dummy(ImVec2(0.0f, 0.0f));
		if (r < 2)
			VSpace(18.0f);
	}

	VSpace(20.0f);
	ButtonOpts save;
	save.kind = ButtonKind::Primary;
	save.width = maxW;
	save.padY = 10.0f;
	ImGui::SetCursorPosX(x0);
	if (Button("Save Profile", save))
	{
		SaveProfile(CalCtx);
		CalCtx.state = CalibrationState::None;
		editView_ = false;
	}
	VSpace(8.0f);
}

// -----------------------------------------------------------------------------------------

namespace
{
	// label | track | value, optional hint below
	bool SliderRow(const char* label, const char* hint, double* value, double minV, double maxV, const char* fmt, float totalW, float labelW, float valueW)
	{
		const float x0 = ImGui::GetCursorPosX();
		const float y0 = ImGui::GetCursorPosY();
		const float rowH = px(20.0f) + 2.0f * px(7.0f);
		const float sliderW = totalW - labelW - 16.0f - 16.0f - valueW;

		float th = TextSize(F.regular, 13.0f, label).y;
		ImGui::SetCursorPos(ImVec2(x0, y0 + (rowH - th) * 0.5f));
		Text(F.regular, 13.0f, P.text, label);

		ImGui::SetCursorPos(ImVec2(x0 + px(labelW + 16.0f), y0 + px(7.0f)));
		char id[64];
		std::snprintf(id, sizeof id, "##slider_%s_%p", label, (void*)value);
		bool changed = Slider(id, value, minV, maxV, sliderW);

		char buf[48];
		std::snprintf(buf, sizeof buf, fmt, *value);
		ImVec2 vs = TextSize(F.mono, 12.5f, buf);
		ImDrawList* dl = ImGui::GetWindowDrawList();
		ImVec2 wp = ImGui::GetWindowPos();
		float sx = ImGui::GetScrollX(), sy = ImGui::GetScrollY();
		DrawText(dl, F.mono, 12.5f, ImVec2(wp.x - sx + x0 + px(totalW) - vs.x, wp.y - sy + y0 + (rowH - vs.y) * 0.5f), P.textStrong, buf);

		ImGui::SetCursorPos(ImVec2(x0, y0 + rowH));
		ImGui::Dummy(ImVec2(0.0f, 0.0f));
		if (hint && *hint)
		{
			ImGui::SetCursorPosX(x0 + px(labelW + 16.0f));
			Hint(hint, totalW - labelW - 16.0f);
			VSpace(6.0f);
		}
		return changed;
	}

	void ParamSliders(protocol::OneEuroParams& p, bool& changed, float totalW)
	{
		changed |= SliderRow("minCutoff", "How steady things look when you are not moving. Lower = calmer image, higher = more responsive (drag right if things feel laggy or floaty).", &p.minCutoff, 0.1, 10.0, "%.3f Hz", totalW, 90.0f, 76.0f);
		changed |= SliderRow("beta", "How quickly the smoothing keeps up when you move fast. Drag right if fast movements feel delayed; drag left if they look shaky.", &p.beta, 0.0, 2.0, "%.3f", totalW, 90.0f, 76.0f);
		changed |= SliderRow("dCutoff", "Fine-tunes how the smoothing reacts as your movement speed changes. Most people can leave this alone.", &p.dCutoff, 0.1, 4.0, "%.3f Hz", totalW, 90.0f, 76.0f);
	}
}

void UserInterface::RenderPreview()
{
	ImVec2 avail = ImGui::GetContentRegionAvail();
	float h = std::max(px(220.0f), avail.y);
	preview_.Render(ImVec2(avail.x, h));
}

void UserInterface::RenderLighthouse()
{
	const float maxW = std::min(820.0f, PageDesignWidth());
	lighthouse::EnsureScanning();

	TextWrapped(F.regular, 13.0f, P.textMuted, maxW,
		"Turn your base stations on, into standby, or to sleep without a Lighthouse headset. "
		"Works with V2 base stations over Bluetooth LE.");
	VSpace(18.0f);

	if (!lighthouse::Available())
	{
		Text(F.regular, 13.0f, P.yellow, "Bluetooth LE is not available on this PC.");
		VSpace(8.0f);
		TextWrapped(F.regular, 12.5f, P.textDim, maxW, "A Bluetooth 4.0+ adapter is required to control base stations.");
		return;
	}

	auto stations = lighthouse::Stations();

	SectionHeader("Base Stations", maxW);
	VSpace(10.0f);

	if (stations.empty())
	{
		Text(F.regular, 12.5f, P.textDim, "Scanning for base stations...");
		VSpace(8.0f);
		TextWrapped(F.regular, 12.5f, P.textDim, maxW, "Make sure the stations have power and are within Bluetooth range.");
		return;
	}

	char buf[128];
	for (const auto& s : stations)
	{
		Text(F.semibold, 13.5f, P.textStrong, s.name.c_str());
		ImGui::SameLine();
		const char* stateText = "Unknown";
		unsigned stateColor = P.textDim;
		switch (s.state)
		{
		case lighthouse::Power::Awake: stateText = "Awake"; stateColor = P.green; break;
		case lighthouse::Power::Standby: stateText = "Standby"; stateColor = P.yellow; break;
		case lighthouse::Power::Sleep: stateText = "Sleeping"; stateColor = P.textMuted; break;
		default: break;
		}
		Text(F.medium, 12.0f, stateColor, stateText);
		ImGui::SameLine();
		std::snprintf(buf, sizeof buf, "%d dBm", s.rssi);
		Text(F.regular, 11.5f, P.textFooter, buf);
		VSpace(8.0f);

		ButtonOpts small;
		small.fontSize = 12.5f;
		small.padX = 14.0f;
		small.padY = 7.0f;
		small.enabled = !s.busy;

		std::snprintf(buf, sizeof buf, "Wake##%llx", (unsigned long long)s.address);
		if (Button(buf, small))
			lighthouse::RequestPower(s.address, lighthouse::Power::Awake);
		ImGui::SameLine();
		std::snprintf(buf, sizeof buf, "Standby##%llx", (unsigned long long)s.address);
		if (Button(buf, small))
			lighthouse::RequestPower(s.address, lighthouse::Power::Standby);
		ImGui::SameLine();
		std::snprintf(buf, sizeof buf, "Sleep##%llx", (unsigned long long)s.address);
		if (Button(buf, small))
			lighthouse::RequestPower(s.address, lighthouse::Power::Sleep);
		ImGui::SameLine();
		std::snprintf(buf, sizeof buf, "Refresh##%llx", (unsigned long long)s.address);
		ButtonOpts ghost = small;
		ghost.kind = ButtonKind::Ghost;
		if (Button(buf, ghost))
			lighthouse::RequestRefresh(s.address);
		if (s.busy)
		{
			ImGui::SameLine();
			Text(F.regular, 12.0f, P.textDim, "working...");
		}
		if (!s.error.empty())
		{
			VSpace(4.0f);
			Text(F.regular, 12.0f, P.danger, s.error.c_str());
		}
		VSpace(14.0f);
		HLine(maxW);
		VSpace(14.0f);
	}

	SectionHeader("All Stations", maxW);
	VSpace(10.0f);
	{
		ButtonOpts opts;
		opts.fontSize = 13.0f;
		if (Button("Wake all"))
			lighthouse::RequestPowerAll(lighthouse::Power::Awake);
		ImGui::SameLine();
		if (Button("Standby all"))
			lighthouse::RequestPowerAll(lighthouse::Power::Standby);
		ImGui::SameLine();
		if (Button("Sleep all"))
			lighthouse::RequestPowerAll(lighthouse::Power::Sleep);
	}
	VSpace(10.0f);
	TextWrapped(F.regular, 12.0f, P.textDim, maxW,
		"Sleeping or standby stations stop tracking immediately. Standby wakes up faster than sleep; older station firmware only supports sleep.");
}

void UserInterface::RenderSmoothing()
{
	const float maxW = std::min(820.0f, PageDesignWidth());
	bool changed = false;

	Text(F.regular, 13.0f, P.green, "NOTE: Changes here take effect instantly, no need to re-calibrate.");
	VSpace(18.0f);

	SectionHeader("Lighthouse Trackers & Controllers", maxW);
	VSpace(10.0f);
	TextWrapped(F.regular, 12.5f, P.textMuted, maxW,
		"Smooths all lighthouse devices (Vive/Tundra trackers, Index controllers) so they show less jittery movement, "
		"for example for dancing or full body recordings. Recommended: 25% - smooth movement while keeping latency minimal. "
		"The higher the percentage, the more latency you get on fast movement. 0% turns it off.");
	VSpace(12.0f);
	{
		double value = CalCtx.lighthouseSmoothing;
		char label[32];
		std::snprintf(label, sizeof label, "%.0f %%", value);
		if (Slider("##lighthouseSmoothing", &value, 0.0, 100.0, maxW - 90.0f))
		{
			CalCtx.lighthouseSmoothing = value;
			changed = true;
		}
		ImGui::SameLine();
		Text(F.medium, 13.0f, P.textStrong, label);
	}
	VSpace(24.0f);

	SectionHeader("Headset Tracker", maxW);
	VSpace(10.0f);
	changed |= CheckboxRow("Smooth headset tracker",
		"Steadies what you see through the headset to reduce shaking (HMD Driven mode only). Adds a tiny bit of delay - if the view feels laggy when you move quickly, adjust the sliders below.",
		&CalCtx.headFilterEnabled, maxW);
	VSpace(10.0f);
	ParamSliders(CalCtx.headFilterParams, changed, maxW);

	if (changed)
	{
		SendOneEuroParams();
		smoothingDirty_ = true;
	}
	if (smoothingDirty_ && !ImGui::IsAnyMouseDown())
	{
		SaveProfile(CalCtx);
		smoothingDirty_ = false;
	}
}

void UserInterface::RenderSettings()
{
	Text(F.regular, 13.0f, P.yellow, "NOTE: Most settings below require re-calibration to be applied");
	VSpace(16.0f);

	const float fullW = PageDesignWidth();
	const float colW = (fullW - 40.0f) * 0.5f;
	const float cardW = (fullW - 2.0f * 12.0f) / 3.0f;

	SectionHeader("Tracking Method", fullW);
	VSpace(12.0f);

	{
		ImDrawList* dl = ImGui::GetWindowDrawList();

		const float padX = 16.0f, padY = 15.0f, dotD = 15.0f, gap = 10.0f;

		auto cardHeight = [&](float cardW, const char* label, const char* hint, const char* req) -> float
		{
			const float innerW = cardW - padX * 2.0f;
			const float headH = std::max(TextSize(F.semibold, 13.5f, label).y, px(dotD));
			return px(padY) + headH + px(9.0f)
				+ TextSize(F.regular, 12.0f, hint, innerW).y + px(7.0f)
				+ TextSize(F.regular, 12.0f, req, innerW).y + px(padY);
		};

		auto methodCard = [&](float cardW, float h, const char* label, const char* tag, unsigned tagColor,
			const char* hint, const char* req, bool active) -> bool
		{
			const float innerW = cardW - padX * 2.0f;

			ImVec2 labelSize = TextSize(F.semibold, 13.5f, label);
			ImVec2 tagSize = TextSize(F.semibold, 11.0f, tag);
			ImVec2 hintSize = TextSize(F.regular, 12.0f, hint, innerW);

			const float headH = std::max(labelSize.y, px(dotD));

			ImVec2 p = ImGui::GetCursorScreenPos();
			ImGui::PushID(label);
			ImGui::InvisibleButton("##method", ImVec2(px(cardW), h));
			ImGui::PopID();
			HoverHand();
			const bool clicked = ImGui::IsItemClicked();

			ImVec2 p1(p.x + px(cardW), p.y + h);
			dl->AddRectFilled(p, p1, Col(active ? P.cardActive : P.inputBg), px(5.0f));
			dl->AddRect(p, p1, Col(active ? P.accent : P.borderStrong), px(5.0f), 0, px(1.0f));

			float cy = p.y + px(padY) + headH * 0.5f;
			ImVec2 c(p.x + px(padX) + px(dotD) * 0.5f, cy);
			dl->AddCircleFilled(c, px(dotD) * 0.5f, Col(active ? P.accent : P.inputBg));
			dl->AddCircle(c, px(dotD) * 0.5f, Col(active ? P.accent : P.checkBorder), 0, px(1.0f));
			if (active)
				dl->AddCircleFilled(c, px(dotD) * 0.5f - px(3.0f), Col(P.cardActive));

			float tx = p.x + px(padX + dotD + gap);
			DrawText(dl, F.semibold, 13.5f, ImVec2(tx, cy - labelSize.y * 0.5f), P.textStrong, label);

			float bx = tx + labelSize.x + px(gap);
			ImVec2 b0(bx, cy - tagSize.y * 0.5f - px(2.0f));
			ImVec2 b1(bx + tagSize.x + px(14.0f), cy + tagSize.y * 0.5f + px(2.0f));
			dl->AddRectFilled(b0, b1, Col(tagColor, 0.12f), px(3.0f));
			DrawText(dl, F.semibold, 11.0f, ImVec2(bx + px(7.0f), b0.y + px(2.0f)), tagColor, tag);

			float ty = p.y + px(padY) + headH + px(9.0f);
			ImGui::SetCursorScreenPos(ImVec2(p.x + px(padX), ty));
			TextWrapped(F.regular, 12.0f, P.textMuted, innerW, hint);
			ImGui::SetCursorScreenPos(ImVec2(p.x + px(padX), ty + hintSize.y + px(7.0f)));
			TextWrapped(F.regular, 12.0f, P.textDim, innerW, req);

			ImGui::SetCursorScreenPos(ImVec2(p.x, p1.y));
			return clicked;
		};

		const float cardsX = ImGui::GetCursorPosX();
		const float rowY = ImGui::GetCursorPosY();

		const char* hintHmd = "The headset owns the tracking space and your lighthouse devices follow it. A tracker on your head keeps them lined up continuously, even when the headset silently re-centres.";
		const char* hintLighthouse = "The tracker on your head owns the tracking space and the headset follows it. Not recommended on Galaxy XR, Vive Pro or Pico 4, where it can cause jitter.";
		const char* hintNoTracker = "The headset owns the tracking space and your lighthouse devices follow it, but there is no running alignment. You calibrate once by holding a controller or a tracker against your head, then put it back.";
		const char* reqTracker = "Requires a tracker on top of your head.";
		const char* reqNone = "No permanent tracker needed. Drifts over time.";

		const float cardH = std::max(cardHeight(cardW, "HMD Driven", hintHmd, reqTracker),
			std::max(cardHeight(cardW, "Lighthouse Driven", hintLighthouse, reqTracker),
				cardHeight(cardW, "HMD Driven + No Tracker", hintNoTracker, reqNone)));

		const bool isHmd = CalCtx.followSlamHmd && !CalCtx.noHeadTracker;
		const bool isNoTracker = CalCtx.followSlamHmd && CalCtx.noHeadTracker;

		bool pickHmd = methodCard(cardW, cardH, "HMD Driven", "Highly Recommended", P.green,
			hintHmd, reqTracker, isHmd);

		ImGui::SetCursorPos(ImVec2(cardsX + px(cardW + 12.0f), rowY));
		bool pickLighthouse = methodCard(cardW, cardH, "Lighthouse Driven", "Recommended", P.yellow,
			hintLighthouse, reqTracker, !CalCtx.followSlamHmd);

		ImGui::SetCursorPos(ImVec2(cardsX + 2.0f * px(cardW + 12.0f), rowY));
		bool pickNoTracker = methodCard(cardW, cardH, "HMD Driven + No Tracker", "Not Recommended", P.danger,
			hintNoTracker, reqNone, isNoTracker);

		ImGui::SetCursorPos(ImVec2(cardsX, rowY + cardH));

		if ((pickHmd && !isHmd) || (pickNoTracker && !isNoTracker) || (pickLighthouse && CalCtx.followSlamHmd))
		{
			CalCtx.followSlamHmd = pickHmd || pickNoTracker;
			CalCtx.noHeadTracker = pickNoTracker;
			if (CalCtx.validProfile)
				SaveProfile(CalCtx);
		}
	}

	VSpace(28.0f);

	const float x0 = ImGui::GetCursorPosX();
	const float y0 = ImGui::GetCursorPosY();

	// Left column
	CheckboxRow("Fallback to SLAM",
		"Temporarily uses the headset's own (SLAM) tracking if the head tracker loses line of sight.",
		&CalCtx.fallbackToSlam, colW);
	CheckboxRow("Enable Angular Velocity",
		"Passes the tracker's angular velocity through to SteamVR. Off by default, it can cause issues with some devices.",
		&CalCtx.enableAngularVelocity, colW);
	CheckboxRow("Relative Calibration",
		"Continuously re-aligns SLAM-tracked devices (controllers etc.) to the calibrated space by comparing the headset's SLAM pose with the tracker-driven pose.",
		&CalCtx.continuousSync, colW);
	if (CheckboxRow("Hide Head Tracker",
		"Parks the tracker mounted on your headset far out of the way so games and SteamVR stop treating it as a device in your play space. Alignment is unaffected. Needs HMD Driven with a tracker, and pauses itself while you calibrate.",
		&CalCtx.hideHeadTracker, colW) && CalCtx.validProfile)
		SaveProfile(CalCtx);

	float leftBottom = ImGui::GetCursorPosY();

	// Right column
	const float rx = x0 + px(colW + 40.0f);
	ImGui::SetCursorPos(ImVec2(rx, y0));
	{
		const float labelW = 110.0f, valueW = 26.0f;
		const float rowH = px(20.0f);
		ImDrawList* dl = ImGui::GetWindowDrawList();
		ImVec2 wp = ImGui::GetWindowPos();

		auto valueSlider = [&](const char* id, const char* label, const char* hint, double* v, double minV, double maxV, const char* fmt, bool* deactivated) -> bool
		{
			float y = ImGui::GetCursorPosY();
			float th = TextSize(F.regular, 13.0f, label).y;
			ImGui::SetCursorPos(ImVec2(rx, y + (rowH - th) * 0.5f));
			Text(F.regular, 13.0f, P.text, label);
			ImGui::SetCursorPos(ImVec2(rx + px(labelW + 14.0f), y));
			bool changed = Slider(id, v, minV, maxV, colW - labelW - 14.0f - 14.0f - valueW);
			if (deactivated)
				*deactivated = ImGui::IsItemDeactivated();
			char buf[16];
			std::snprintf(buf, sizeof buf, fmt, *v);
			ImVec2 vs = TextSize(F.mono, 12.5f, buf);
			DrawText(dl, F.mono, 12.5f, ImVec2(wp.x - ImGui::GetScrollX() + rx + px(colW) - vs.x, wp.y - ImGui::GetScrollY() + y + (rowH - vs.y) * 0.5f), P.textStrong, buf);
			ImGui::SetCursorPos(ImVec2(rx, y + rowH));
			VSpace(8.0f);
			ImGui::SetCursorPosX(rx);
			Hint(hint, colW);
			VSpace(24.0f);
			return changed;
		};

		double pred = CalCtx.predictionTime;
		if (valueSlider("##prediction", "Prediction Time", "How many frames of prediction SteamVR applies to the tracker. Some wireless solutions may need more prediction to feel smooth.", &pred, 0.0, 10.0, "%.1f", nullptr))
			CalCtx.predictionTime = (float)pred;

		{
			const float scales[4] = { 1.0f, 1.25f, 1.5f, 2.0f };
			const float y = ImGui::GetCursorPosY();
			const float pillH = TextSize(F.mono, 12.0f, "0").y + px(10.0f);
			const float th = TextSize(F.regular, 13.0f, "UI Scale").y;

			ImGui::SetCursorPos(ImVec2(rx, y + (pillH - th) * 0.5f));
			Text(F.regular, 13.0f, P.text, "UI Scale");

			float x = rx + px(labelW + 14.0f);
			for (int i = 0; i < 4; i++)
			{
				char buf[8];
				std::snprintf(buf, sizeof buf, "%.2f", scales[i]);
				const bool active = CalCtx.uiScale > scales[i] - 0.001f && CalCtx.uiScale < scales[i] + 0.001f;

				ImGui::SetCursorPos(ImVec2(x, y));
				if (Pill(buf, active) && !active)
				{
					CalCtx.uiScale = scales[i];
					if (CalCtx.validProfile)
						SaveProfile(CalCtx);
				}
				x += TextSize(F.mono, 12.0f, buf).x + 2.0f * px(10.0f) + px(6.0f);
			}

			ImGui::SetCursorPos(ImVec2(rx, y + pillH));
			VSpace(8.0f);
			ImGui::SetCursorPosX(rx);
			Hint("Size of text and controls in this window and in the SteamVR dashboard overlay.", colW);
			VSpace(24.0f);
		}

		ImGui::SetCursorPosX(rx);
		Text(F.regular, 13.0f, P.textMuted, "Calibration Speed");
		VSpace(10.0f);

		struct SpeedOpt { const char* label; const char* hint; CalibrationContext::Speed speed; };
		const SpeedOpt speeds[3] = {
			{ "Fast", "One pass through the look-around sequence. Quickest option, but small mistakes during calibration show up as inaccuracy.", CalibrationContext::FAST },
			{ "Slow", "Recommended. Two passes through the look-around sequence, so the same samples cover more directions.", CalibrationContext::SLOW },
			{ "Very Slow", "Three passes and the most samples. Use this when you want the most precise result.", CalibrationContext::VERY_SLOW },
		};
		for (int i = 0; i < 3; i++)
		{
			ImGui::SetCursorPosX(rx);
			if (RadioRow(speeds[i].label, speeds[i].hint, CalCtx.calibrationSpeed == speeds[i].speed, colW))
				CalCtx.calibrationSpeed = speeds[i].speed;
		}
	}

	float rightBottom = ImGui::GetCursorPosY();
	ImGui::SetCursorPos(ImVec2(x0, std::max(leftBottom, rightBottom)));
	VSpace(8.0f);
}

// -----------------------------------------------------------------------------------------

void UserInterface::RenderWizard()
{
	const char* popupId = "##calibration_wizard";
	if (wizardOpen_ && !ImGui::IsPopupOpen(popupId))
		ImGui::OpenPopup(popupId);
	if (!wizardOpen_)
		return;

	const float w = 460.0f;
	ImGuiIO& io = ImGui::GetIO();
	ImGui::SetNextWindowPos(ImVec2(io.DisplaySize.x * 0.5f, io.DisplaySize.y * 0.5f), ImGuiCond_Always, ImVec2(0.5f, 0.5f));
	ImGui::SetNextWindowSize(ImVec2(px(w), 0.0f));
	ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(0.0f, 0.0f));
	ImGui::PushStyleVar(ImGuiStyleVar_WindowRounding, px(6.0f));
	ImGui::PushStyleVar(ImGuiStyleVar_WindowBorderSize, 1.0f);
	ImGui::PushStyleColor(ImGuiCol_Border, ColV(P.borderButton));
	ImGui::PushStyleColor(ImGuiCol_PopupBg, ColV(P.card));

	const ImGuiWindowFlags flags = ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove |
		ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoSavedSettings | ImGuiWindowFlags_NoScrollbar;

	if (ImGui::BeginPopupModal(popupId, nullptr, flags))
	{
		ImDrawList* dl = ImGui::GetWindowDrawList();
		const ImVec2 wp = ImGui::GetWindowPos();
		const float ww = px(w);

		// --- state ---
		const CalibrationState st = CalCtx.state;
		const bool running = st == CalibrationState::Begin || st == CalibrationState::Detect || st == CalibrationState::Sampling;
		float fraction = 0.0f;
		std::string lastLine;
		CalibrationFeedback(fraction, lastLine);

		std::string counter, title;
		Icon icon = Icon::Dot;
		unsigned iconColor = P.link, fillColor = P.accent;
		float fill = 0.0f;
		bool done = false, failed = false;

		if (st == CalibrationState::Begin)
		{
			counter = "Starting";
			title = "Starting calibration";
			icon = Icon::Ring;
		}
		else if (st == CalibrationState::Detect)
		{
			counter = "Detecting tracker";
			title = "Move your head around";
			icon = Icon::Ring;
			fill = fraction;
		}
		else if (st == CalibrationState::Sampling)
		{
			const int total = CalCtx.sequenceSteps > 0 ? CalCtx.sequenceSteps : CalCtx.SequenceStepCount();
			const int step = std::min(total - 1, std::max(0, CalCtx.sequenceStep));
			char buf[32];
			std::snprintf(buf, sizeof buf, "Step %d of %d", step + 1, total);
			counter = buf;
			title = kSteps[step % kStepCount].title;
			icon = kSteps[step % kStepCount].icon;
			fill = fraction;
		}
		else
		{
			if (CalCtx.lastCalibrationOk)
			{
				done = true;
				counter = "Complete";
				title = "Done";
				icon = Icon::Check;
				iconColor = P.green;
				fillColor = P.green;
				fill = 1.0f;
			}
			else
			{
				failed = true;
				counter = "Aborted";
				title = "Calibration failed";
				icon = Icon::Cross;
				iconColor = P.danger;
				fillColor = P.danger;
				fill = 0.0f;
			}
		}

		// --- header ---
		{
			float h = px(14.0f) + TextSize(F.semibold, 13.0f, "X").y + px(14.0f);
			ImVec2 p = ImGui::GetCursorScreenPos();
			ImGui::Dummy(ImVec2(ww, h));
			DrawText(dl, F.semibold, 13.0f, ImVec2(p.x + px(18.0f), p.y + px(14.0f)), P.textStrong, "Calibration");
			ImVec2 cs = TextSize(F.regular, 12.0f, counter.c_str());
			DrawText(dl, F.regular, 12.0f, ImVec2(p.x + ww - px(18.0f) - cs.x, p.y + px(14.0f) + (TextSize(F.semibold, 13.0f, "X").y - cs.y) * 0.5f), P.textDim, counter.c_str());
			dl->AddRectFilled(ImVec2(p.x, p.y + h - 1.0f), ImVec2(p.x + ww, p.y + h), Col(P.border));
		}

		// --- body ---
		{
			const bool hasMessage = (failed || (running && st == CalibrationState::Sampling && !lastLine.empty() && lastLine.find("samples") != std::string::npos));
			float bodyH = px(34.0f + 44.0f + 20.0f + 19.0f + 26.0f + 3.0f + 30.0f) + (hasMessage ? px(10.0f + 12.0f) : 0.0f);
			ImVec2 p = ImGui::GetCursorScreenPos();
			ImGui::Dummy(ImVec2(ww, bodyH));
			float cx = p.x + ww * 0.5f;
			float y = p.y + px(34.0f);
			DrawIcon(dl, icon, ImVec2(cx, y + px(22.0f)), 40.0f, Col(iconColor), 3.0f);
			y += px(44.0f + 20.0f);
			DrawTextCentered(dl, F.semibold, 19.0f, ImVec2(cx, y + px(9.5f)), P.textBright, title.c_str());
			y += px(19.0f + 26.0f);
			float bx0 = p.x + px(24.0f), bx1 = p.x + ww - px(24.0f);
			dl->AddRectFilled(ImVec2(bx0, y), ImVec2(bx1, y + px(3.0f)), Col(P.borderStrong), px(2.0f));
			if (fill > 0.0f)
				dl->AddRectFilled(ImVec2(bx0, y), ImVec2(bx0 + (bx1 - bx0) * std::min(1.0f, fill), y + px(3.0f)), Col(fillColor), px(2.0f));
			y += px(3.0f);
			if (hasMessage)
			{
				y += px(10.0f);
				DrawTextCentered(dl, F.regular, 12.0f, ImVec2(cx, y + px(6.0f)), failed ? P.danger : P.textDim, lastLine.c_str());
			}
		}

		// --- footer ---
		{
			ButtonOpts secondary;
			secondary.padX = 16.0f;
			secondary.padY = 8.0f;
			float bh = TextSize(F.medium, 13.0f, "X").y + px(16.0f);
			float h = px(14.0f) + bh + px(14.0f);
			ImVec2 p = ImGui::GetCursorScreenPos();
			dl->AddRectFilled(ImVec2(p.x, p.y), ImVec2(p.x + ww, p.y + 1.0f), Col(P.border));

			ImGui::SetCursorScreenPos(ImVec2(p.x + px(18.0f), p.y + px(14.0f)));
			if (Button(running ? "Cancel" : "Close", secondary))
			{
				if (running)
					CancelCalibration();
				wizardOpen_ = false;
				ImGui::CloseCurrentPopup();
			}
			if (done)
			{
				ButtonOpts primary;
				primary.kind = ButtonKind::Primary;
				primary.padX = 18.0f;
				primary.padY = 8.0f;
				float dw = TextSize(F.semibold, 13.0f, "Done").x + px(36.0f);
				ImGui::SetCursorScreenPos(ImVec2(p.x + ww - px(18.0f) - dw, p.y + px(14.0f)));
				if (Button("Done", primary))
				{
					wizardOpen_ = false;
					ImGui::CloseCurrentPopup();
				}
			}
			ImGui::SetCursorScreenPos(ImVec2(p.x, p.y + h));
			ImGui::Dummy(ImVec2(ww, 0.0f));
		}

		ImGui::EndPopup();
	}

	ImGui::PopStyleColor(2);
	ImGui::PopStyleVar(3);
}

void UserInterface::RenderConfirm()
{
	const char* popupId = "##confirm_remove";
	if (confirmRemove_ && !ImGui::IsPopupOpen(popupId))
		ImGui::OpenPopup(popupId);
	if (!confirmRemove_)
		return;

	const float w = 380.0f;
	ImGuiIO& io = ImGui::GetIO();
	ImGui::SetNextWindowPos(ImVec2(io.DisplaySize.x * 0.5f, io.DisplaySize.y * 0.5f), ImGuiCond_Always, ImVec2(0.5f, 0.5f));
	ImGui::SetNextWindowSize(ImVec2(px(w), 0.0f));
	ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(px(20.0f), px(20.0f)));
	ImGui::PushStyleVar(ImGuiStyleVar_WindowRounding, px(6.0f));
	ImGui::PushStyleVar(ImGuiStyleVar_WindowBorderSize, 1.0f);
	ImGui::PushStyleColor(ImGuiCol_Border, ColV(P.borderButton));
	ImGui::PushStyleColor(ImGuiCol_PopupBg, ColV(P.card));

	const ImGuiWindowFlags flags = ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove |
		ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoSavedSettings | ImGuiWindowFlags_NoScrollbar;

	if (ImGui::BeginPopupModal(popupId, nullptr, flags))
	{
		const float inner = w - 40.0f;
		Text(F.semibold, 14.0f, P.textBright, "Remove Calibration?");
		VSpace(8.0f);
		TextWrapped(F.regular, 13.0f, P.textMuted, inner, "All offsets will be discarded. You will need to calibrate again.");
		VSpace(20.0f);

		ButtonOpts cancel;
		cancel.padX = 16.0f;
		cancel.padY = 8.0f;
		ButtonOpts remove;
		remove.kind = ButtonKind::Danger;
		remove.padX = 16.0f;
		remove.padY = 8.0f;
		float cw = TextSize(F.medium, 13.0f, "Cancel").x + px(32.0f);
		float rw = TextSize(F.semibold, 13.0f, "Remove").x + px(32.0f);
		float x0 = ImGui::GetCursorPosX();
		ImGui::SetCursorPosX(x0 + px(inner) - cw - px(8.0f) - rw);
		if (Button("Cancel", cancel))
		{
			confirmRemove_ = false;
			ImGui::CloseCurrentPopup();
		}
		ImGui::SameLine(0.0f, px(8.0f));
		if (Button("Remove", remove))
		{
			CalCtx.Clear();
			SaveProfile(CalCtx);
			confirmRemove_ = false;
			ImGui::CloseCurrentPopup();
		}
		ImGui::Dummy(ImVec2(px(inner), 0.0f));
		ImGui::EndPopup();
	}

	ImGui::PopStyleColor(2);
	ImGui::PopStyleVar(3);
}
