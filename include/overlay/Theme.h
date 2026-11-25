// SPDX-License-Identifier: AGPL-3.0-only
// Added by Shinyflvres, 2026-08-23. Part of SpaceSync, a modified version of OpenVR-SpaceOverride by Nyabsi (AGPL-3.0). See NOTICE.md

#pragma once

#include <imgui.h>

namespace ui
{
	struct Palette
	{
		unsigned pageBg       = 0x1a1e29;
		unsigned card         = 0x17171a;
		unsigned cardActive   = 0x212125;
		unsigned titleBar     = 0x111113;
		unsigned border       = 0x232326;
		unsigned borderStrong = 0x2c2c31;
		unsigned borderButton = 0x34343a;
		unsigned inputBg      = 0x0e0e10;
		unsigned button       = 0x242428;
		unsigned buttonHover  = 0x2c2c31;
		unsigned stepHover    = 0x323238;
		unsigned accent       = 0xe4e4e8;
		unsigned accentHover  = 0xffffff;
		unsigned textOnAccent = 0x16161a;
		unsigned textBright   = 0xf4f4f6;
		unsigned textStrong   = 0xe8e8eb;
		unsigned text         = 0xd7d7db;
		unsigned textTitle    = 0xcdcdd2;
		unsigned textButton   = 0xa9a9b0;
		unsigned textMuted    = 0x8f8f97;
		unsigned textDim      = 0x78787f;
		unsigned textIcon     = 0x7f7f87;
		unsigned textFooter   = 0x606068;
		unsigned textDisabled = 0x6b6b73;
		unsigned green        = 0xb587ff;
		unsigned link         = 0xc9c9cf;
		unsigned linkHover    = 0xffffff;
		unsigned danger       = 0xb23b3b;
		unsigned dangerHover  = 0xc04343;
		unsigned dangerBorder = 0x4b3236;
		unsigned yellow       = 0xffa387;
		unsigned knob         = 0xffffff;
		unsigned checkBorder  = 0x44444b;
		unsigned rowRule      = 0x1e1e21;
		unsigned overlay      = 0x0c0e11;
		unsigned ring[5]      = { 0xffa19d, 0xf5bcd6, 0xe8c9f5, 0xc4b0f7, 0x9480f0 };
		float ringStops[5]    = { 0.0f, 0.30f, 0.52f, 0.72f, 1.0f };
	};

	struct Fonts
	{
		ImFont* regular = nullptr;
		ImFont* medium = nullptr;
		ImFont* semibold = nullptr;
		ImFont* bold = nullptr;
		ImFont* mono = nullptr;
		ImFont* monoMedium = nullptr;
	};

	extern Palette P;
	extern Fonts F;

	void Init(float displayScale, bool srgbFramebuffer);

	void SetDisplayScale(float scale);
	void SetContentScale(float scale);
	float DisplayScale();
	float ContentScale();
	float S();
	float px(float designPx);

	ImU32 Col(unsigned rgb, float alpha = 1.0f);
	ImVec4 ColV(unsigned rgb, float alpha = 1.0f);

	float FontPx(float designSize);
	void PushFont(ImFont* font, float designSize);
	void PopFont();
	ImVec2 TextSize(ImFont* font, float designSize, const char* text, float wrapDesignWidth = 0.0f);
	void Text(ImFont* font, float designSize, unsigned rgb, const char* text);
	void TextWrapped(ImFont* font, float designSize, unsigned rgb, float wrapDesignWidth, const char* text);
	void DrawText(ImDrawList* dl, ImFont* font, float designSize, ImVec2 pos, unsigned rgb, const char* text);
	void DrawTextCentered(ImDrawList* dl, ImFont* font, float designSize, ImVec2 center, unsigned rgb, const char* text);

	enum class Icon { Check, Ring, Dot, ArrowLeft, ArrowRight, ArrowUp, ArrowDown, Cross, Minus, Plus, Square };
	void DrawIcon(ImDrawList* dl, Icon icon, ImVec2 center, float designSize, ImU32 col, float designThickness = 1.6f);
	void DrawGradientRing(ImDrawList* dl, ImVec2 center, float outerRadius, float thickness);

	bool TabItem(const char* label, bool active);

	enum class ButtonKind { Secondary, Primary, Danger, Ghost };
	struct ButtonOpts
	{
		ButtonKind kind = ButtonKind::Secondary;
		float padX = 18.0f, padY = 9.0f;
		float minWidth = 0.0f;
		float width = 0.0f;
		float fontSize = 13.0f;
		ImFont* font = nullptr;
		bool enabled = true;
		unsigned hoverBorder = 0;
	};
	bool Button(const char* label, const ButtonOpts& opts = {});
	bool Link(const char* label, float fontSize = 13.0f);

	bool Checkbox(const char* label, bool* value, float rowPadY = 0.0f);
	bool Radio(const char* label, bool active);

	bool CheckboxRow(const char* label, const char* hint, bool* value, float designWidth, bool enabled = true);
	bool RadioRow(const char* label, const char* hint, bool active, float designWidth);
	bool DropdownRow(const char* label, const char* hint, int* index, const char* const* items, int count, float designWidth);
	void Hint(const char* text, float designWidth);

	bool Slider(const char* id, double* value, double minValue, double maxValue, float designWidth);
	bool Stepper(const char* id, double* value, double step, int decimals, float designWidth, bool enabled = true);
	bool Pill(const char* label, bool active, ImFont* font = nullptr, float fontSize = 12.0f);

	void SectionHeader(const char* label, float designWidth);
	void HLine(float designWidth, unsigned rgb = 0);
	void VSpace(float designPx);

	bool HoverHand();
}
