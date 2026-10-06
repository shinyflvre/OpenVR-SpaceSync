// SPDX-License-Identifier: AGPL-3.0-only
// Added by Shinyflvres, 2026-08-23. Part of SpaceSync, a modified version of OpenVR-SpaceOverride by Nyabsi (AGPL-3.0). See NOTICE.md

#include "Theme.h"
#include "EmbeddedFiles.h"
#include "Localization.h"

#include <imgui_internal.h>

#include <cmath>
#include <cstdio>
#include <cfloat>
#include <algorithm>

namespace ui
{
	Palette P;
	Fonts F;

	static float g_displayScale = 1.0f;
	static float g_contentScale = 1.0f;
	static bool g_srgb = false;

	void SetDisplayScale(float scale) { g_displayScale = (scale > 0.0f) ? scale : 1.0f; }
	void SetContentScale(float scale) { g_contentScale = (scale > 0.0f) ? scale : 1.0f; }
	float DisplayScale() { return g_displayScale; }
	float ContentScale() { return g_contentScale; }
	float S() { return g_displayScale * g_contentScale; }
	float px(float designPx) { return designPx * S(); }

	static float ToLinear(float c)
	{
		return c <= 0.04045f ? c / 12.92f : std::pow((c + 0.055f) / 1.055f, 2.4f);
	}

	ImVec4 ColV(unsigned rgb, float alpha)
	{
		float r = ((rgb >> 16) & 0xff) / 255.0f;
		float g = ((rgb >> 8) & 0xff) / 255.0f;
		float b = (rgb & 0xff) / 255.0f;
		if (g_srgb)
		{
			r = ToLinear(r);
			g = ToLinear(g);
			b = ToLinear(b);
		}
		return ImVec4(r, g, b, alpha);
	}

	ImU32 Col(unsigned rgb, float alpha)
	{
		return ImGui::ColorConvertFloat4ToU32(ColV(rgb, alpha));
	}

	void Init(float displayScale, bool srgbFramebuffer)
	{
		g_displayScale = displayScale;
		g_srgb = srgbFramebuffer;

		ImGuiIO& io = ImGui::GetIO();

		ImFontConfig cfg;
		cfg.PixelSnapH = false;
		cfg.OversampleH = 2;
		cfg.OversampleV = 2;

		const float base = FontPx(13.0f);

		const char* jaCandidates[] = { "C:\\Windows\\Fonts\\YuGothM.ttc", "C:\\Windows\\Fonts\\YuGothR.ttc", "C:\\Windows\\Fonts\\meiryo.ttc" };
		const char* scCandidates[] = { "C:\\Windows\\Fonts\\msyh.ttc", "C:\\Windows\\Fonts\\msyhl.ttc", "C:\\Windows\\Fonts\\simhei.ttf" };
		const char* koCandidates[] = { "C:\\Windows\\Fonts\\malgun.ttf", "C:\\Windows\\Fonts\\malgunsl.ttf" };
		const char* jaFont = nullptr;
		const char* scFont = nullptr;
		const char* koFont = nullptr;
		for (const char* p : jaCandidates) { FILE* f = fopen(p, "rb"); if (f) { fclose(f); jaFont = p; break; } }
		for (const char* p : scCandidates) { FILE* f = fopen(p, "rb"); if (f) { fclose(f); scFont = p; break; } }
		for (const char* p : koCandidates) { FILE* f = fopen(p, "rb"); if (f) { fclose(f); koFont = p; break; } }

		ImFontConfig cjk;
		cjk.MergeMode = true;
		cjk.PixelSnapH = false;
		cjk.OversampleH = 2;
		cjk.OversampleV = 2;

		auto addWithCjk = [&](const void* data, int size) {
			ImFont* font = io.Fonts->AddFontFromMemoryCompressedTTF(data, size, base, &cfg);
			if (jaFont) io.Fonts->AddFontFromFileTTF(jaFont, base, &cjk);
			if (scFont) io.Fonts->AddFontFromFileTTF(scFont, base, &cjk);
			if (koFont) io.Fonts->AddFontFromFileTTF(koFont, base, &cjk);
			return font;
		};

		F.regular = addWithCjk(Manrope400_compressed_data, (int)Manrope400_compressed_size);
		F.medium = addWithCjk(Manrope500_compressed_data, (int)Manrope500_compressed_size);
		F.semibold = addWithCjk(Manrope600_compressed_data, (int)Manrope600_compressed_size);
		F.bold = addWithCjk(Manrope700_compressed_data, (int)Manrope700_compressed_size);
		F.mono = addWithCjk(JetBrainsMono400_compressed_data, (int)JetBrainsMono400_compressed_size);
		F.monoMedium = addWithCjk(JetBrainsMono500_compressed_data, (int)JetBrainsMono500_compressed_size);
		io.FontDefault = F.regular;

		ImGuiStyle& style = ImGui::GetStyle();
		style = ImGuiStyle();
		style.WindowPadding = ImVec2(px(10.0f), px(8.0f));
		style.WindowRounding = 0.0f;
		style.WindowBorderSize = 0.0f;
		style.PopupRounding = px(4.0f);
		style.PopupBorderSize = 1.0f;
		style.FramePadding = ImVec2(px(6.0f), px(4.0f));
		style.ItemSpacing = ImVec2(0.0f, 0.0f);
		style.ItemInnerSpacing = ImVec2(0.0f, 0.0f);
		style.ScrollbarSize = px(10.0f);
		style.ScrollbarRounding = px(5.0f);
		style.FontSizeBase = base;

		ImVec4* c = style.Colors;
		c[ImGuiCol_Text] = ColV(P.text);
		c[ImGuiCol_TextDisabled] = ColV(P.textDisabled);
		c[ImGuiCol_WindowBg] = ColV(P.card);
		c[ImGuiCol_ChildBg] = ColV(P.card, 0.0f);
		c[ImGuiCol_PopupBg] = ColV(P.card);
		c[ImGuiCol_Border] = ColV(P.borderButton);
		c[ImGuiCol_BorderShadow] = ImVec4(0, 0, 0, 0);
		c[ImGuiCol_FrameBg] = ColV(P.inputBg);
		c[ImGuiCol_FrameBgHovered] = ColV(P.button);
		c[ImGuiCol_FrameBgActive] = ColV(P.buttonHover);
		c[ImGuiCol_ScrollbarBg] = ColV(P.card, 0.0f);
		c[ImGuiCol_ScrollbarGrab] = ColV(P.borderButton);
		c[ImGuiCol_ScrollbarGrabHovered] = ColV(P.stepHover);
		c[ImGuiCol_ScrollbarGrabActive] = ColV(P.stepHover);
		c[ImGuiCol_Button] = ColV(P.button);
		c[ImGuiCol_ButtonHovered] = ColV(P.buttonHover);
		c[ImGuiCol_ButtonActive] = ColV(P.stepHover);
		c[ImGuiCol_ModalWindowDimBg] = ColV(P.overlay, 0.9f);
		c[ImGuiCol_NavCursor] = ColV(P.accent, 0.0f);
	}

	float FontPx(float designSize)
	{
		float v = px(designSize);
		float r = std::floor(v + 0.5f);
		return r < 1.0f ? 1.0f : r;
	}

	void PushFont(ImFont* font, float designSize)
	{
		ImGui::PushFont(font ? font : F.regular, FontPx(designSize));
	}

	void PopFont()
	{
		ImGui::PopFont();
	}

	ImVec2 TextSize(ImFont* font, float designSize, const char* text, float wrapDesignWidth)
	{
		text = loc::tr(text);
		PushFont(font, designSize);
		ImVec2 size = ImGui::CalcTextSize(text, nullptr, false, wrapDesignWidth > 0.0f ? px(wrapDesignWidth) : -1.0f);
		PopFont();
		return size;
	}

	void Text(ImFont* font, float designSize, unsigned rgb, const char* text)
	{
		text = loc::tr(text);
		ImVec2 c = ImGui::GetCursorScreenPos();
		ImGui::SetCursorScreenPos(ImVec2(std::floor(c.x + 0.5f), std::floor(c.y + 0.5f)));
		PushFont(font, designSize);
		ImGui::PushStyleColor(ImGuiCol_Text, ColV(rgb));
		ImGui::TextUnformatted(text);
		ImGui::PopStyleColor();
		PopFont();
	}

	void TextWrapped(ImFont* font, float designSize, unsigned rgb, float wrapDesignWidth, const char* text)
	{
		text = loc::tr(text);
		ImVec2 c = ImGui::GetCursorScreenPos();
		ImGui::SetCursorScreenPos(ImVec2(std::floor(c.x + 0.5f), std::floor(c.y + 0.5f)));
		PushFont(font, designSize);
		ImGui::PushStyleColor(ImGuiCol_Text, ColV(rgb));
		ImGui::PushTextWrapPos(ImGui::GetCursorPosX() + px(wrapDesignWidth));
		ImGui::TextUnformatted(text);
		ImGui::PopTextWrapPos();
		ImGui::PopStyleColor();
		PopFont();
	}

	void DrawText(ImDrawList* dl, ImFont* font, float designSize, ImVec2 pos, unsigned rgb, const char* text)
	{
		text = loc::tr(text);
		dl->AddText(font ? font : F.regular, FontPx(designSize), ImVec2(std::floor(pos.x + 0.5f), std::floor(pos.y + 0.5f)), Col(rgb), text);
	}

	void DrawTextCentered(ImDrawList* dl, ImFont* font, float designSize, ImVec2 center, unsigned rgb, const char* text)
	{
		text = loc::tr(text);
		ImFont* f = font ? font : F.regular;
		const float fs = FontPx(designSize);
		ImVec2 size = f->CalcTextSizeA(fs, FLT_MAX, 0.0f, text);
		dl->AddText(f, fs, ImVec2(std::floor(center.x - size.x * 0.5f + 0.5f), std::floor(center.y - size.y * 0.5f + 0.5f)), Col(rgb), text);
	}

	void DrawIcon(ImDrawList* dl, Icon icon, ImVec2 c, float designSize, ImU32 col, float designThickness)
	{
		const float s = px(designSize);
		const float t = px(designThickness);
		switch (icon)
		{
		case Icon::Check:
		{
			ImVec2 p[3] = {
				ImVec2(c.x - 0.36f * s, c.y + 0.02f * s),
				ImVec2(c.x - 0.10f * s, c.y + 0.28f * s),
				ImVec2(c.x + 0.40f * s, c.y - 0.30f * s)
			};
			dl->AddPolyline(p, 3, col, 0, t);
			break;
		}
		case Icon::Ring:
			dl->AddCircle(c, s * 0.5f, col, 0, t);
			break;
		case Icon::Dot:
			dl->AddCircleFilled(c, s * 0.5f, col);
			break;
		case Icon::ArrowLeft:
		case Icon::ArrowRight:
		case Icon::ArrowUp:
		case Icon::ArrowDown:
		{
			float a = icon == Icon::ArrowRight ? 0.0f : icon == Icon::ArrowDown ? 1.5707963f : icon == Icon::ArrowLeft ? 3.1415926f : -1.5707963f;
			auto R = [&](float x, float y) {
				return ImVec2(c.x + (x * std::cos(a) - y * std::sin(a)) * s, c.y + (x * std::sin(a) + y * std::cos(a)) * s);
			};
			dl->AddLine(R(-0.45f, 0.0f), R(0.45f, 0.0f), col, t);
			dl->AddLine(R(0.45f, 0.0f), R(0.10f, -0.33f), col, t);
			dl->AddLine(R(0.45f, 0.0f), R(0.10f, 0.33f), col, t);
			break;
		}
		case Icon::Cross:
			dl->AddLine(ImVec2(c.x - 0.35f * s, c.y - 0.35f * s), ImVec2(c.x + 0.35f * s, c.y + 0.35f * s), col, t);
			dl->AddLine(ImVec2(c.x - 0.35f * s, c.y + 0.35f * s), ImVec2(c.x + 0.35f * s, c.y - 0.35f * s), col, t);
			break;
		case Icon::Minus:
			dl->AddLine(ImVec2(c.x - 0.4f * s, c.y), ImVec2(c.x + 0.4f * s, c.y), col, t);
			break;
		case Icon::Plus:
			dl->AddLine(ImVec2(c.x - 0.4f * s, c.y), ImVec2(c.x + 0.4f * s, c.y), col, t);
			dl->AddLine(ImVec2(c.x, c.y - 0.4f * s), ImVec2(c.x, c.y + 0.4f * s), col, t);
			break;
		case Icon::Square:
			dl->AddRect(ImVec2(c.x - 0.38f * s, c.y - 0.38f * s), ImVec2(c.x + 0.38f * s, c.y + 0.38f * s), col, 0.0f, 0, t);
			break;
		}
	}

	static unsigned LerpRgb(unsigned a, unsigned b, float t)
	{
		auto ch = [&](int shift) {
			float x = (float)((a >> shift) & 0xff), y = (float)((b >> shift) & 0xff);
			int v = (int)std::lround(x + (y - x) * t);
			return (unsigned)std::min(255, std::max(0, v)) << shift;
		};
		return ch(16) | ch(8) | ch(0);
	}

	static unsigned RingColor(float t)
	{
		t = std::min(1.0f, std::max(0.0f, t));
		for (int i = 0; i < 4; i++)
		{
			if (t <= P.ringStops[i + 1])
			{
				float span = P.ringStops[i + 1] - P.ringStops[i];
				return LerpRgb(P.ring[i], P.ring[i + 1], span > 0.0f ? (t - P.ringStops[i]) / span : 0.0f);
			}
		}
		return P.ring[4];
	}

	void DrawGradientRing(ImDrawList* dl, ImVec2 c, float outerRadius, float thickness)
	{
		const int n = 256;
		const float feather = 1.0f;
		const float radius[4] = { outerRadius + feather * 0.5f, outerRadius - feather * 0.5f, outerRadius - thickness + feather * 0.5f, outerRadius - thickness - feather * 0.5f };
		const float alpha[4] = { 0.0f, 1.0f, 1.0f, 0.0f };
		const float lineLength = 2.0f * outerRadius * 1.41421356f;
		const ImVec2 dir(-0.70710678f, 0.70710678f);
		const ImVec2 uv = dl->_Data->TexUvWhitePixel;

		dl->PrimReserve(n * 18, n * 4);
		const unsigned int base = dl->_VtxCurrentIdx;
		for (int i = 0; i < n; i++)
		{
			float a = 6.28318531f * (float)i / (float)n;
			ImVec2 u(std::cos(a), std::sin(a));
			for (int k = 0; k < 4; k++)
			{
				ImVec2 pos(c.x + u.x * radius[k], c.y + u.y * radius[k]);
				float t = 0.5f + ((pos.x - c.x) * dir.x + (pos.y - c.y) * dir.y) / lineLength;
				dl->PrimWriteVtx(pos, uv, Col(RingColor(t), alpha[k]));
			}
		}
		for (int i = 0; i < n; i++)
		{
			int j = (i + 1) % n;
			for (int b = 0; b < 3; b++)
			{
				ImDrawIdx v0 = (ImDrawIdx)(base + i * 4 + b), v1 = (ImDrawIdx)(base + i * 4 + b + 1);
				ImDrawIdx w0 = (ImDrawIdx)(base + j * 4 + b), w1 = (ImDrawIdx)(base + j * 4 + b + 1);
				dl->PrimWriteIdx(v0); dl->PrimWriteIdx(w0); dl->PrimWriteIdx(w1);
				dl->PrimWriteIdx(v0); dl->PrimWriteIdx(w1); dl->PrimWriteIdx(v1);
			}
		}
	}

	bool HoverHand()
	{
		if (ImGui::IsItemHovered())
		{
			ImGui::SetMouseCursor(ImGuiMouseCursor_Hand);
			return true;
		}
		return false;
	}

	bool TabItem(const char* label, bool active)
	{
		ImFont* font = active ? F.semibold : F.medium;
		ImVec2 text = TextSize(font, 13.0f, label);
		ImVec2 size(text.x + px(28.0f), text.y + px(22.0f));

		ImVec2 p = ImGui::GetCursorScreenPos();
		ImGui::InvisibleButton(label, size);
		bool hovered = HoverHand();
		bool clicked = ImGui::IsItemClicked();

		ImDrawList* dl = ImGui::GetWindowDrawList();
		unsigned color = active || hovered ? P.textStrong : P.textMuted;
		DrawText(dl, font, 13.0f, ImVec2(p.x + px(14.0f), p.y + px(11.0f)), color, label);
		if (active)
			dl->AddRectFilled(ImVec2(p.x, p.y + size.y - px(2.0f)), ImVec2(p.x + size.x, p.y + size.y), Col(P.accent));
		return clicked;
	}

	bool Button(const char* label, const ButtonOpts& o)
	{
		ImFont* font = o.font;
		if (!font)
			font = (o.kind == ButtonKind::Primary || o.kind == ButtonKind::Danger) ? F.semibold : (o.kind == ButtonKind::Ghost ? F.regular : F.medium);

		ImVec2 text = TextSize(font, o.fontSize, label);
		float w = o.width > 0.0f ? px(o.width) : std::max(text.x + 2.0f * px(o.padX), px(o.minWidth));
		float h = text.y + 2.0f * px(o.padY);

		ImVec2 p = ImGui::GetCursorScreenPos();
		ImGui::InvisibleButton(label, ImVec2(w, h));
		bool hovered = o.enabled && ImGui::IsItemHovered();
		if (hovered)
			ImGui::SetMouseCursor(ImGuiMouseCursor_Hand);
		bool clicked = o.enabled && ImGui::IsItemClicked();

		ImDrawList* dl = ImGui::GetWindowDrawList();
		float r = px(4.0f);
		unsigned textColor = P.text;
		switch (o.kind)
		{
		case ButtonKind::Secondary:
			dl->AddRectFilled(p, ImVec2(p.x + w, p.y + h), Col(hovered ? P.buttonHover : P.button), r);
			dl->AddRect(p, ImVec2(p.x + w, p.y + h), Col(!o.enabled ? P.borderStrong : (hovered && o.hoverBorder ? o.hoverBorder : P.borderButton)), r);
			textColor = o.enabled ? P.text : P.textDisabled;
			break;
		case ButtonKind::Primary:
			dl->AddRectFilled(p, ImVec2(p.x + w, p.y + h), Col(hovered ? P.accentHover : P.accent), r);
			textColor = P.textOnAccent;
			break;
		case ButtonKind::Danger:
			dl->AddRectFilled(p, ImVec2(p.x + w, p.y + h), Col(hovered ? P.dangerHover : P.danger), r);
			textColor = 0xffffff;
			break;
		case ButtonKind::Ghost:
			textColor = hovered ? P.linkHover : P.link;
			break;
		}
		DrawText(dl, font, o.fontSize, ImVec2(p.x + (w - text.x) * 0.5f, p.y + px(o.padY)), textColor, label);
		return clicked;
	}

	bool Link(const char* label, float fontSize)
	{
		ButtonOpts o;
		o.kind = ButtonKind::Ghost;
		o.padX = 0.0f;
		o.padY = 2.0f;
		o.fontSize = fontSize;
		return Button(label, o);
	}

	bool Checkbox(const char* label, bool* value, float rowPadY)
	{
		const float box = px(16.0f);
		const float gap = px(11.0f);
		ImVec2 text = TextSize(F.regular, 13.0f, label);
		float h = std::max(box, text.y) + 2.0f * px(rowPadY);
		float w = box + gap + text.x;

		ImVec2 p = ImGui::GetCursorScreenPos();
		ImGui::InvisibleButton(label, ImVec2(w, h));
		HoverHand();
		bool clicked = ImGui::IsItemClicked();
		if (clicked)
			*value = !*value;

		ImDrawList* dl = ImGui::GetWindowDrawList();
		float cy = p.y + h * 0.5f;
		ImVec2 b0(p.x, cy - box * 0.5f), b1(p.x + box, cy + box * 0.5f);
		float r = px(3.0f);
		if (*value)
		{
			dl->AddRectFilled(b0, b1, Col(P.accent), r);
			DrawIcon(dl, Icon::Check, ImVec2((b0.x + b1.x) * 0.5f, cy), 11.0f, Col(P.textOnAccent), 1.8f);
		}
		else
		{
			dl->AddRectFilled(b0, b1, Col(P.inputBg), r);
			dl->AddRect(b0, b1, Col(P.checkBorder), r);
		}
		DrawText(dl, F.regular, 13.0f, ImVec2(p.x + box + gap, cy - text.y * 0.5f), P.text, label);
		return clicked;
	}

	bool Radio(const char* label, bool active)
	{
		const float d = px(16.0f);
		const float gap = px(11.0f);
		ImVec2 text = TextSize(F.regular, 13.0f, label);
		float h = std::max(d, text.y);
		float w = d + gap + text.x;

		ImVec2 p = ImGui::GetCursorScreenPos();
		ImGui::InvisibleButton(label, ImVec2(w, h));
		HoverHand();
		bool clicked = ImGui::IsItemClicked();

		ImDrawList* dl = ImGui::GetWindowDrawList();
		ImVec2 c(p.x + d * 0.5f, p.y + h * 0.5f);
		if (active)
		{
			dl->AddCircleFilled(c, d * 0.5f, Col(P.accent));
			dl->AddCircleFilled(c, d * 0.5f - px(1.0f), Col(P.card));
			dl->AddCircleFilled(c, d * 0.5f - px(4.0f), Col(P.accent));
		}
		else
		{
			dl->AddCircleFilled(c, d * 0.5f, Col(P.inputBg));
			dl->AddCircle(c, d * 0.5f, Col(P.checkBorder), 0, 1.0f);
		}
		DrawText(dl, F.regular, 13.0f, ImVec2(p.x + d + gap, c.y - text.y * 0.5f), P.text, label);
		return clicked;
	}

	void Hint(const char* text, float designWidth)
	{
		TextWrapped(F.regular, 12.0f, P.textDim, designWidth, text);
	}

	static bool OptionRow(const char* label, const char* hint, bool isRadio, bool on, float designWidth)
	{
		hint = loc::tr(hint);
		const float box = px(16.0f);
		const float gap = px(11.0f);
		const float padY = px(11.0f);
		const float w = px(designWidth);
		const float textW = designWidth - 16.0f - 11.0f;

		ImVec2 labelSize = TextSize(F.regular, 13.0f, label);
		ImVec2 hintSize = (hint && *hint) ? TextSize(F.regular, 12.0f, hint, textW) : ImVec2(0, 0);
		float textH = labelSize.y + ((hint && *hint) ? px(3.0f) + hintSize.y : 0.0f);
		float h = padY + std::max(box, textH) + padY;

		ImVec2 p = ImGui::GetCursorScreenPos();
		ImGui::PushID(label);
		ImGui::InvisibleButton("##row", ImVec2(w, h));
		ImGui::PopID();
		HoverHand();
		bool clicked = ImGui::IsItemClicked();

		ImDrawList* dl = ImGui::GetWindowDrawList();
		float cy = p.y + padY + labelSize.y * 0.5f;
		if (isRadio)
		{
			ImVec2 c(p.x + box * 0.5f, cy);
			if (on)
			{
				dl->AddCircleFilled(c, box * 0.5f, Col(P.accent));
				dl->AddCircleFilled(c, box * 0.5f - px(1.0f), Col(P.card));
				dl->AddCircleFilled(c, box * 0.5f - px(4.0f), Col(P.accent));
			}
			else
			{
				dl->AddCircleFilled(c, box * 0.5f, Col(P.inputBg));
				dl->AddCircle(c, box * 0.5f, Col(P.checkBorder), 0, 1.0f);
			}
		}
		else
		{
			ImVec2 b0(p.x, cy - box * 0.5f), b1(p.x + box, cy + box * 0.5f);
			float r = px(3.0f);
			if (on)
			{
				dl->AddRectFilled(b0, b1, Col(P.accent), r);
				DrawIcon(dl, Icon::Check, ImVec2((b0.x + b1.x) * 0.5f, cy), 11.0f, Col(P.textOnAccent), 1.8f);
			}
			else
			{
				dl->AddRectFilled(b0, b1, Col(P.inputBg), r);
				dl->AddRect(b0, b1, Col(P.checkBorder), r);
			}
		}

		float tx = p.x + box + gap;
		DrawText(dl, F.regular, 13.0f, ImVec2(tx, p.y + padY), P.text, label);
		if (hint && *hint)
			dl->AddText(F.regular, px(12.0f), ImVec2(tx, p.y + padY + labelSize.y + px(3.0f)), Col(P.textDim), hint, nullptr, px(textW));

		dl->AddRectFilled(ImVec2(p.x, p.y + h - 1.0f), ImVec2(p.x + w, p.y + h), Col(P.rowRule));
		return clicked;
	}

	bool CheckboxRow(const char* label, const char* hint, bool* value, float designWidth)
	{
		bool clicked = OptionRow(label, hint, false, *value, designWidth);
		if (clicked)
			*value = !*value;
		return clicked;
	}

	bool RadioRow(const char* label, const char* hint, bool active, float designWidth)
	{
		return OptionRow(label, hint, true, active, designWidth);
	}

	bool DropdownRow(const char* label, const char* hint, int* index, const char* const* items, int count, float designWidth)
	{
		hint = loc::tr(hint);
		const float comboW = 170.0f;
		const float padY = px(11.0f);
		const float w = px(designWidth);
		const float textW = designWidth - comboW - 16.0f;

		ImVec2 labelSize = TextSize(F.regular, 13.0f, label);
		ImVec2 hintSize = (hint && *hint) ? TextSize(F.regular, 12.0f, hint, textW) : ImVec2(0, 0);
		float textH = labelSize.y + ((hint && *hint) ? px(3.0f) + hintSize.y : 0.0f);
		const float itemFont = 13.0f;
		const float itemTextH = TextSize(F.medium, itemFont, "Ag").y;
		const float frameH = itemTextH + 2.0f * px(9.0f);
		float h = padY + std::max(frameH, textH) + padY;

		ImVec2 p = ImGui::GetCursorScreenPos();
		ImDrawList* dl = ImGui::GetWindowDrawList();
		DrawText(dl, F.regular, 13.0f, ImVec2(p.x, p.y + padY), P.text, label);
		if (hint && *hint)
			dl->AddText(F.regular, px(12.0f), ImVec2(p.x, p.y + padY + labelSize.y + px(3.0f)), Col(P.textDim), hint, nullptr, px(textW));

		int current = (*index >= 0 && *index < count) ? *index : 0;
		bool changed = false;
		const float r = px(4.0f);
		const float fw = px(comboW);

		auto chevron = [](ImDrawList* d, ImVec2 c, bool up, ImU32 col) {
			float s = px(4.0f), t = px(1.5f), dir = up ? -1.0f : 1.0f;
			d->AddLine(ImVec2(c.x - s, c.y - dir * s * 0.5f), ImVec2(c.x, c.y + dir * s * 0.5f), col, t);
			d->AddLine(ImVec2(c.x, c.y + dir * s * 0.5f), ImVec2(c.x + s, c.y - dir * s * 0.5f), col, t);
		};

		ImGui::PushID(label);
		const bool wasOpen = ImGui::IsPopupOpen("##list");
		ImVec2 f0(p.x + w - fw, p.y + padY);
		ImGui::SetCursorScreenPos(f0);
		ImGui::InvisibleButton("##frame", ImVec2(fw, frameH));
		bool hovered = HoverHand();
		if (ImGui::IsItemClicked() && !wasOpen)
			ImGui::OpenPopup("##list");
		bool active = hovered || wasOpen;
		ImVec2 f1(f0.x + fw, f0.y + frameH);
		dl->AddRectFilled(f0, f1, Col(active ? P.buttonHover : P.button), r);
		dl->AddRect(f0, f1, Col(P.borderButton), r);
		DrawText(dl, F.medium, itemFont, ImVec2(f0.x + px(12.0f), f0.y + px(9.0f)), P.text, items[current]);
		chevron(dl, ImVec2(f1.x - px(16.0f), f0.y + frameH * 0.5f), wasOpen, Col(active ? P.textBright : P.textButton));

		const float rowH = itemTextH + 2.0f * px(8.0f);
		const float listPad = px(4.0f);
		ImGui::SetNextWindowPos(ImVec2(f0.x, f1.y + px(4.0f)));
		ImGui::SetNextWindowSize(ImVec2(fw, rowH * count + 2.0f * listPad));
		ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(0.0f, 0.0f));
		ImGui::PushStyleVar(ImGuiStyleVar_ItemSpacing, ImVec2(0.0f, 0.0f));
		ImGui::PushStyleVar(ImGuiStyleVar_PopupRounding, r);
		ImGui::PushStyleVar(ImGuiStyleVar_PopupBorderSize, 1.0f);
		ImGui::PushStyleColor(ImGuiCol_PopupBg, ColV(P.button));
		ImGui::PushStyleColor(ImGuiCol_Border, ColV(P.borderButton));
		if (ImGui::BeginPopup("##list", ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse))
		{
			ImDrawList* pl = ImGui::GetWindowDrawList();
			ImVec2 w0 = ImGui::GetWindowPos();
			for (int i = 0; i < count; i++)
			{
				ImGui::SetCursorScreenPos(ImVec2(w0.x, w0.y + listPad + rowH * i));
				ImVec2 r0 = ImGui::GetCursorScreenPos();
				ImGui::PushID(i);
				ImGui::InvisibleButton("##item", ImVec2(fw, rowH));
				ImGui::PopID();
				bool rowHovered = HoverHand();
				bool selected = i == current;
				if (rowHovered)
					pl->AddRectFilled(ImVec2(r0.x + listPad, r0.y), ImVec2(r0.x + fw - listPad, r0.y + rowH), Col(P.stepHover), r);
				DrawText(pl, F.medium, itemFont, ImVec2(r0.x + px(12.0f), r0.y + px(8.0f)), selected || rowHovered ? P.textBright : P.text, items[i]);
				if (selected)
					DrawIcon(pl, Icon::Check, ImVec2(r0.x + fw - px(16.0f), r0.y + rowH * 0.5f), 11.0f, Col(P.textBright), 1.6f);
				if (ImGui::IsItemClicked())
				{
					if (i != current)
					{
						*index = i;
						changed = true;
					}
					ImGui::CloseCurrentPopup();
				}
			}
			ImGui::SetCursorScreenPos(ImVec2(w0.x, w0.y + listPad + rowH * count));
			ImGui::Dummy(ImVec2(fw, listPad));
			ImGui::EndPopup();
		}
		ImGui::PopStyleColor(2);
		ImGui::PopStyleVar(4);
		ImGui::PopID();

		ImGui::SetCursorScreenPos(p);
		ImGui::Dummy(ImVec2(w, h));
		dl->AddRectFilled(ImVec2(p.x, p.y + h - 1.0f), ImVec2(p.x + w, p.y + h), Col(P.rowRule));
		return changed;
	}

	bool Slider(const char* id, double* value, double minValue, double maxValue, float designWidth)
	{
		const float w = px(designWidth);
		const float h = px(20.0f);
		const float knob = px(14.0f);

		ImVec2 p = ImGui::GetCursorScreenPos();
		ImGui::InvisibleButton(id, ImVec2(w, h));
		bool hovered = HoverHand();
		bool changed = false;
		if (ImGui::IsItemActive())
		{
			float mx = ImGui::GetIO().MousePos.x;
			double t = (mx - p.x) / w;
			t = std::min(1.0, std::max(0.0, t));
			double v = minValue + t * (maxValue - minValue);
			if (v != *value)
			{
				*value = v;
				changed = true;
			}
		}

		double frac = (maxValue > minValue) ? (*value - minValue) / (maxValue - minValue) : 0.0;
		frac = std::min(1.0, std::max(0.0, frac));

		ImDrawList* dl = ImGui::GetWindowDrawList();
		float cy = p.y + h * 0.5f;
		float th = px(4.0f);
		dl->AddRectFilled(ImVec2(p.x, cy - th * 0.5f), ImVec2(p.x + w, cy + th * 0.5f), Col(P.borderStrong), th * 0.5f);
		float kx = p.x + (float)frac * w;
		dl->AddRectFilled(ImVec2(p.x, cy - th * 0.5f), ImVec2(kx, cy + th * 0.5f), Col(P.accent), th * 0.5f);
		dl->AddCircleFilled(ImVec2(kx, cy), knob * 0.5f, Col(hovered || ImGui::IsItemActive() ? 0xffffff : P.knob));
		return changed;
	}

	bool Stepper(const char* id, double* value, double step, int decimals, float designWidth, bool enabled)
	{
		const float btnW = px(28.0f), btnH = px(34.0f), gap = px(5.0f);
		const float boxW = px(designWidth) - 2.0f * (btnW + gap);
		bool changed = false;

		ImGui::PushID(id);
		ImDrawList* dl = ImGui::GetWindowDrawList();

		ImVec2 p = ImGui::GetCursorScreenPos();
		ImGui::InvisibleButton("value", ImVec2(boxW, btnH));
		dl->AddRectFilled(p, ImVec2(p.x + boxW, p.y + btnH), Col(P.inputBg), px(4.0f));
		dl->AddRect(p, ImVec2(p.x + boxW, p.y + btnH), Col(P.borderStrong, enabled ? 1.0f : 0.5f), px(4.0f));
		char buf[64];
		std::snprintf(buf, sizeof buf, "%.*f", decimals, *value);
		ImVec2 ts = TextSize(F.mono, 13.0f, buf);
		dl->AddText(F.mono, px(13.0f), ImVec2(p.x + px(10.0f), p.y + (btnH - ts.y) * 0.5f), Col(enabled ? P.textStrong : P.textDisabled), buf);

		for (int i = 0; i < 2; i++)
		{
			ImGui::SameLine(0.0f, gap);
			ImVec2 b = ImGui::GetCursorScreenPos();
			ImGui::InvisibleButton(i == 0 ? "dec" : "inc", ImVec2(btnW, btnH));
			bool hovered = enabled && HoverHand();
			if (enabled && ImGui::IsItemClicked())
			{
				*value += (i == 0 ? -step : step);
				changed = true;
			}
			dl->AddRectFilled(b, ImVec2(b.x + btnW, b.y + btnH), Col(hovered ? P.stepHover : P.button, enabled ? 1.0f : 0.5f), px(4.0f));
			dl->AddRect(b, ImVec2(b.x + btnW, b.y + btnH), Col(P.borderButton, enabled ? 1.0f : 0.5f), px(4.0f));
			DrawIcon(dl, i == 0 ? Icon::Minus : Icon::Plus, ImVec2(b.x + btnW * 0.5f, b.y + btnH * 0.5f), 11.0f, Col(hovered ? 0xffffff : (enabled ? P.textButton : P.textDisabled), enabled ? 1.0f : 0.6f), 1.4f);
		}

		ImGui::PopID();
		return changed;
	}

	bool Pill(const char* label, bool active, ImFont* font, float fontSize)
	{
		if (!font) font = F.mono;
		ImVec2 text = TextSize(font, fontSize, label);
		ImVec2 size(text.x + 2.0f * px(10.0f), text.y + 2.0f * px(5.0f));
		ImVec2 p = ImGui::GetCursorScreenPos();
		ImGui::InvisibleButton(label, size);
		bool hovered = HoverHand();
		bool clicked = ImGui::IsItemClicked();

		ImDrawList* dl = ImGui::GetWindowDrawList();
		float r = px(4.0f);
		dl->AddRectFilled(p, ImVec2(p.x + size.x, p.y + size.y), Col(active ? P.accent : (hovered ? P.buttonHover : P.button)), r);
		dl->AddRect(p, ImVec2(p.x + size.x, p.y + size.y), Col(active ? P.accent : P.borderButton), r);
		DrawText(dl, font, fontSize, ImVec2(p.x + px(10.0f), p.y + px(5.0f)), active ? P.textOnAccent : P.textMuted, label);
		return clicked;
	}

	void HLine(float designWidth, unsigned rgb)
	{
		ImVec2 p = ImGui::GetCursorScreenPos();
		ImGui::GetWindowDrawList()->AddRectFilled(p, ImVec2(p.x + px(designWidth), p.y + 1.0f), Col(rgb ? rgb : P.border));
		ImGui::Dummy(ImVec2(px(designWidth), 1.0f));
	}

	void VSpace(float designPx)
	{
		ImGui::Dummy(ImVec2(0.0f, px(designPx)));
	}

	void SectionHeader(const char* label, float designWidth)
	{
		Text(F.semibold, 12.5f, P.textMuted, label);
		VSpace(9.0f);
		HLine(designWidth);
		VSpace(10.0f);
	}
}
