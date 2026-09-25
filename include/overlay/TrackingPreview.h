// SPDX-License-Identifier: AGPL-3.0-only
// Added by Shinyflvres, 2026-09-25. Part of SpaceSync, a modified version of OpenVR-SpaceOverride by Nyabsi (AGPL-3.0). See NOTICE.md

#pragma once

#include <imgui.h>

class TrackingPreview
{
public:
	void Render(ImVec2 size);

private:
	float camYaw = 0.7f;
	float camPitch = 0.32f;
	float camDist = 3.4f;
	float targetX = 0.0f, targetY = 1.0f, targetZ = 0.0f;
	bool targetInit = false;
};
