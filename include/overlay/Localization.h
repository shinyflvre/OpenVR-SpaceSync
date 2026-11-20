// SPDX-License-Identifier: AGPL-3.0-only
// Added by Shinyflvres, 2026-09-26. Part of SpaceSync, a modified version of OpenVR-SpaceOverride by Nyabsi (AGPL-3.0). See NOTICE.md

#pragma once

namespace loc
{
	enum class Lang { English = 0, Japanese = 1, Chinese = 2, Korean = 3 };

	void SetLanguage(Lang lang);
	Lang Current();
	const char* tr(const char* english);
}
