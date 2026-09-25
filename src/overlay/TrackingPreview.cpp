// SPDX-License-Identifier: AGPL-3.0-only
// Added by Shinyflvres, 2026-09-25. Part of SpaceSync, a modified version of OpenVR-SpaceOverride by Nyabsi (AGPL-3.0). See NOTICE.md

#include "TrackingPreview.h"
#include "Calibration.h"
#include "Theme.h"

#include <openvr.h>
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

using namespace ui;

namespace
{
	struct V3 { float x = 0, y = 0, z = 0; };

	V3 operator+(V3 a, V3 b) { return { a.x + b.x, a.y + b.y, a.z + b.z }; }
	V3 operator-(V3 a, V3 b) { return { a.x - b.x, a.y - b.y, a.z - b.z }; }
	V3 operator*(V3 a, float s) { return { a.x * s, a.y * s, a.z * s }; }
	float Dot(V3 a, V3 b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
	V3 Cross(V3 a, V3 b) { return { a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x }; }
	float Len(V3 a) { return std::sqrt(Dot(a, a)); }
	V3 Norm(V3 a, V3 fallback = { 0, 1, 0 }) { float l = Len(a); return l > 1e-6f ? a * (1.0f / l) : fallback; }

	struct Cam
	{
		V3 pos, r, u, f;
		float focal = 1.0f;
		ImVec2 center;
	};

	bool Project(const Cam& c, V3 p, ImVec2& out, float* depth = nullptr)
	{
		V3 d = p - c.pos;
		float z = Dot(d, c.f);
		if (z < 0.05f)
			return false;
		out = ImVec2(c.center.x + Dot(d, c.r) * c.focal / z, c.center.y - Dot(d, c.u) * c.focal / z);
		if (depth) *depth = z;
		return true;
	}

	void Line3(ImDrawList* dl, const Cam& c, V3 a, V3 b, ImU32 col, float th)
	{
		ImVec2 pa, pb;
		if (Project(c, a, pa) && Project(c, b, pb))
			dl->AddLine(pa, pb, col, th);
	}

	std::string DeviceProp(vr::IVRSystem* sys, uint32_t id, vr::ETrackedDeviceProperty prop)
	{
		char buf[vr::k_unMaxPropertyStringSize] = {};
		vr::ETrackedPropertyError err = vr::TrackedProp_Success;
		uint32_t n = sys->GetStringTrackedDeviceProperty(id, prop, buf, sizeof buf, &err);
		return (err == vr::TrackedProp_Success && n > 1) ? std::string(buf) : std::string();
	}

	std::string TrackerRoleName(const std::string& controllerType)
	{
		struct Entry { const char* type; const char* name; };
		static const Entry kRoles[] = {
			{ "vive_tracker_waist", "Waist" },
			{ "vive_tracker_chest", "Chest" },
			{ "vive_tracker_left_foot", "Left foot" },
			{ "vive_tracker_right_foot", "Right foot" },
			{ "vive_tracker_left_knee", "Left knee" },
			{ "vive_tracker_right_knee", "Right knee" },
			{ "vive_tracker_left_elbow", "Left elbow" },
			{ "vive_tracker_right_elbow", "Right elbow" },
			{ "vive_tracker_left_shoulder", "Left shoulder" },
			{ "vive_tracker_right_shoulder", "Right shoulder" },
			{ "vive_tracker_left_wrist", "Left wrist" },
			{ "vive_tracker_right_wrist", "Right wrist" },
			{ "vive_tracker_left_ankle", "Left ankle" },
			{ "vive_tracker_right_ankle", "Right ankle" },
			{ "vive_tracker_handed", "Hand tracker" },
			{ "vive_tracker_camera", "Camera" },
			{ "vive_tracker_keyboard", "Keyboard" },
		};
		for (const Entry& e : kRoles)
			if (controllerType == e.type)
				return e.name;
		return std::string();
	}

	struct Marker
	{
		V3 pos;
		V3 fwd;
		std::string label;
		unsigned color;
		float size;
	};

	const unsigned kTrackerCol = 0x66d68f;
	const unsigned kControllerCol = 0xe87ab8;
	const unsigned kHmdCol = 0x7ec8f0;
	const unsigned kGrid = 0x2c313c;
	const unsigned kGridAxis = 0x3a4150;
	const unsigned kViewportBg = 0x14161d;
}

void TrackingPreview::Render(ImVec2 size)
{
	ImDrawList* dl = ImGui::GetWindowDrawList();
	ImVec2 p0 = ImGui::GetCursorScreenPos();
	ImVec2 p1 = ImVec2(p0.x + size.x, p0.y + size.y);
	ImGuiIO& io = ImGui::GetIO();

	ImGui::InvisibleButton("##trackingPreview", size);
	bool hovered = ImGui::IsItemHovered();

	if (ImGui::IsItemActive() && ImGui::IsMouseDragging(ImGuiMouseButton_Left, 0.0f))
	{
		camYaw -= io.MouseDelta.x * 0.011f;
		camPitch += io.MouseDelta.y * 0.009f;
		camPitch = std::max(0.03f, std::min(1.25f, camPitch));
	}
	if (hovered && io.MouseWheel != 0.0f)
	{
		camDist *= std::exp(-io.MouseWheel * 0.15f);
		camDist = std::max(1.4f, std::min(7.0f, camDist));
	}

	dl->AddRectFilled(p0, p1, Col(kViewportBg), px(12.0f));
	dl->AddRect(p0, p1, Col(P.border), px(12.0f));
	dl->PushClipRect(p0, p1, true);

	vr::IVRSystem* sys = vr::VRSystem();
	vr::TrackedDevicePose_t poses[vr::k_unMaxTrackedDeviceCount] = {};
	bool hmdValid = false;
	if (sys)
	{
		sys->GetDeviceToAbsoluteTrackingPose(vr::TrackingUniverseStanding, 0.0f, poses, vr::k_unMaxTrackedDeviceCount);
		hmdValid = poses[vr::k_unTrackedDeviceIndex_Hmd].bPoseIsValid;
	}

	if (!hmdValid)
	{
		DrawTextCentered(dl, F.regular, 13.0f, ImVec2((p0.x + p1.x) * 0.5f, (p0.y + p1.y) * 0.5f), P.textMuted, "Waiting for SteamVR tracking...");
		dl->PopClipRect();
		return;
	}

	std::vector<Marker> markers;
	for (uint32_t id = 0; id < vr::k_unMaxTrackedDeviceCount; id++)
	{
		if (!poses[id].bPoseIsValid)
			continue;
		vr::ETrackedDeviceClass cls = sys->GetTrackedDeviceClass(id);
		const vr::HmdMatrix34_t& m = poses[id].mDeviceToAbsoluteTracking;
		V3 pos = { m.m[0][3], m.m[1][3], m.m[2][3] };
		V3 fwd = { -m.m[0][2], -m.m[1][2], -m.m[2][2] };

		if (cls == vr::TrackedDeviceClass_HMD)
		{
			markers.push_back({ pos, fwd, "HMD", kHmdCol, 4.8f });
		}
		else if (cls == vr::TrackedDeviceClass_Controller)
		{
			vr::ETrackedControllerRole role = sys->GetControllerRoleForTrackedDeviceIndex(id);
			std::string label = "Controller";
			if (role == vr::TrackedControllerRole_LeftHand) label = "Left controller";
			else if (role == vr::TrackedControllerRole_RightHand) label = "Right controller";
			markers.push_back({ pos, fwd, label, kControllerCol, 4.2f });
		}
		else if (cls == vr::TrackedDeviceClass_GenericTracker)
		{
			std::string serial = DeviceProp(sys, id, vr::Prop_SerialNumber_String);
			std::string label;
			if (!CalCtx.trackerSerial.empty() && serial == CalCtx.trackerSerial)
				label = "Head tracker";
			else
			{
				label = TrackerRoleName(DeviceProp(sys, id, vr::Prop_ControllerType_String));
				if (label.empty())
					label = serial.size() >= 4 ? "Tracker " + serial.substr(serial.size() - 4) : "Tracker";
			}
			markers.push_back({ pos, fwd, label, kTrackerCol, 4.2f });
		}
	}

	V3 focus = { 0.0f, 1.0f, 0.0f };
	{
		const vr::HmdMatrix34_t& hm = poses[vr::k_unTrackedDeviceIndex_Hmd].mDeviceToAbsoluteTracking;
		focus = { hm.m[0][3], 1.0f, hm.m[2][3] };
	}
	if (!targetInit) { targetX = focus.x; targetY = focus.y; targetZ = focus.z; targetInit = true; }
	float tb = std::min(1.0f, io.DeltaTime * 1.5f);
	targetX += (focus.x - targetX) * tb;
	targetY += (focus.y - targetY) * tb;
	targetZ += (focus.z - targetZ) * tb;

	Cam cam;
	V3 target = { targetX, targetY, targetZ };
	cam.pos = target + V3{ std::cos(camPitch) * std::sin(camYaw), std::sin(camPitch), std::cos(camPitch) * std::cos(camYaw) } * camDist;
	cam.f = Norm(target - cam.pos, { 0, 0, -1 });
	cam.r = Norm(Cross(cam.f, { 0, 1, 0 }), { 1, 0, 0 });
	cam.u = Cross(cam.r, cam.f);
	cam.focal = 0.5f * size.y / std::tan(0.42f);
	cam.center = ImVec2((p0.x + p1.x) * 0.5f, (p0.y + p1.y) * 0.5f);

	const float ext = 3.0f;
	float gx = target.x - std::fmod(target.x, 0.5f);
	float gz = target.z - std::fmod(target.z, 0.5f);
	for (int i = -6; i <= 6; i++)
	{
		float c = i * 0.5f;
		float fade = 1.0f - std::fabs(c) / (ext + 0.8f);
		Line3(dl, cam, { gx + c, 0, gz - ext }, { gx + c, 0, gz + ext }, Col(std::fabs(gx + c) < 0.01f ? kGridAxis : kGrid, 0.55f * fade + (std::fabs(gx + c) < 0.01f ? 0.25f : 0.0f)), 1.0f);
		Line3(dl, cam, { gx - ext, 0, gz + c }, { gx + ext, 0, gz + c }, Col(std::fabs(gz + c) < 0.01f ? kGridAxis : kGrid, 0.55f * fade + (std::fabs(gz + c) < 0.01f ? 0.25f : 0.0f)), 1.0f);
	}

	if (CalCtx.chaperone.valid && CalCtx.chaperone.playSpaceSize.v[0] > 0.1f)
	{
		float hx = CalCtx.chaperone.playSpaceSize.v[0] * 0.5f;
		float hz = CalCtx.chaperone.playSpaceSize.v[1] * 0.5f;
		V3 c0 = { -hx, 0.002f, -hz }, c1 = { hx, 0.002f, -hz }, c2 = { hx, 0.002f, hz }, c3 = { -hx, 0.002f, hz };
		ImU32 pc = Col(P.accent, 0.35f);
		Line3(dl, cam, c0, c1, pc, 1.5f);
		Line3(dl, cam, c1, c2, pc, 1.5f);
		Line3(dl, cam, c2, c3, pc, 1.5f);
		Line3(dl, cam, c3, c0, pc, 1.5f);
	}

	std::vector<int> order(markers.size());
	for (int i = 0; i < (int)order.size(); i++) order[i] = i;
	std::sort(order.begin(), order.end(), [&](int a, int b) {
		return Dot(markers[a].pos - cam.pos, cam.f) > Dot(markers[b].pos - cam.pos, cam.f);
	});

	for (int idx : order)
	{
		const Marker& m = markers[idx];

		ImVec2 pts[20];
		int n = 0;
		for (int i = 0; i < 20; i++)
		{
			float a = (float)i / 20.0f * 6.2831853f;
			ImVec2 s;
			if (Project(cam, { m.pos.x + std::cos(a) * 0.055f, 0.003f, m.pos.z + std::sin(a) * 0.055f }, s))
				pts[n++] = s;
		}
		if (n >= 3)
			dl->AddConvexPolyFilled(pts, n, Col(0x000000, 0.20f));

		ImVec2 s;
		float z;
		if (!Project(cam, m.pos, s, &z))
			continue;
		float scale = std::min(1.6f, 2.2f / z);

		Line3(dl, cam, m.pos, m.pos + m.fwd * 0.09f, Col(m.color, 0.65f), px(1.6f));

		dl->AddCircleFilled(s, px(m.size * 2.2f) * scale, Col(m.color, 0.10f), 20);
		dl->AddCircleFilled(s, px(m.size) * scale, Col(m.color, 0.95f), 20);
		dl->AddCircle(s, px(m.size) * scale, Col(0xffffff, 0.35f), 20, 1.0f);
		DrawText(dl, F.regular, 10.5f, ImVec2(s.x + px(9.0f), s.y - px(14.0f)), P.textMuted, m.label.c_str());
	}

	char counts[64];
	std::snprintf(counts, sizeof counts, "%d devices tracked", (int)markers.size());
	DrawText(dl, F.regular, 10.5f, ImVec2(p0.x + px(14.0f), p1.y - px(24.0f)), P.textDim, counts);
	{
		const char* hint = "Drag to orbit  \xc2\xb7  Scroll to zoom";
		ImVec2 ts = TextSize(F.regular, 10.5f, hint);
		DrawText(dl, F.regular, 10.5f, ImVec2(p1.x - ts.x - px(14.0f), p1.y - px(24.0f)), P.textDim, hint);
	}

	dl->PopClipRect();
}
