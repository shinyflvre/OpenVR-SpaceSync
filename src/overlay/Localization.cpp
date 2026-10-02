// SPDX-License-Identifier: AGPL-3.0-only
// Added by Shinyflvres, 2026-09-26. Part of SpaceSync, a modified version of OpenVR-SpaceOverride by Nyabsi (AGPL-3.0). See NOTICE.md

#include "Localization.h"

#include <string>
#include <unordered_map>

namespace loc
{
	namespace
	{
		struct Entry
		{
			const char* en;
			const char* ja;
			const char* zh;
		};

		const Entry kTable[] = {
			{ "SteamVR Running", "SteamVR 起動中", "SteamVR 运行中" },
			{ "SteamVR Not Running", "SteamVR 未起動", "SteamVR 未运行" },

			{ "Calibration", "キャリブレーション", "校准" },
			{ "Tracking Preview", "トラッキングプレビュー", "追踪预览" },
			{ "Smoothing", "スムージング", "平滑" },
			{ "Basestations", "ベースステーション", "基站" },
			{ "Settings", "設定", "设置" },

			{ "Calibrate", "キャリブレーション", "开始校准" },
			{ "Calibrated", "キャリブレーション済み", "已校准" },
			{ "Click to start", "クリックして開始", "点击开始" },
			{ "Click to calibrate again", "クリックで再キャリブレーション", "点击重新校准" },
			{ "Start SteamVR to calibrate", "キャリブレーションには SteamVR を起動してください", "请先启动 SteamVR 再校准" },
			{ "not detected", "未検出", "未检测到" },
			{ "Edit Calibration", "キャリブレーションを編集", "编辑校准" },
			{ "Remove Calibration", "キャリブレーションを削除", "删除校准" },
			{ "Save Profile", "プロファイルを保存", "保存配置" },
			{ "Back", "戻る", "返回" },
			{ "No calibration yet", "キャリブレーション未実施", "尚未校准" },
			{ "Click the circle to calibrate", "円をクリックしてキャリブレーション", "点击圆圈开始校准" },
			{ "SpaceSync driver not loaded", "SpaceSync ドライバー未読み込み", "SpaceSync 驱动未加载" },
			{ "Restart SteamVR and enable the SpaceSync add-on", "SteamVR を再起動し、SpaceSync アドオンを有効にしてください", "请重启 SteamVR 并启用 SpaceSync 插件" },
			{ "Headset tracker not connected", "ヘッドセットトラッカー未接続", "头显追踪器未连接" },
			{ "Calibrated without a head tracker", "ヘッドトラッカーなしでキャリブレーション済み", "在没有头部追踪器的情况下校准" },
			{ "Calibrate again with the tracker on your head", "頭にトラッカーを付けて再キャリブレーションしてください", "请把追踪器戴在头上重新校准" },
			{ "Not active yet without a head tracker. Calibrate once so SpaceSync can measure the delay.",
			  "ヘッドトラッカーなしではまだ有効ではありません。遅延を測定するため、一度キャリブレーションしてください。",
			  "没有头部追踪器时尚未启用。请校准一次，让 SpaceSync 测量延迟。" },
			{ "HMD Driven, one-time calibration", "HMD 主導・ワンタイムキャリブレーション", "头显主导・一次性校准" },
			{ "no running alignment", "連続アライメントなし", "无持续对齐" },
			{ "Override disabled", "オーバーライド無効", "覆盖已禁用" },
			{ "HMD tracking system changed?", "HMD のトラッキング方式が変わった可能性", "头显追踪系统可能已更改" },
			{ "HMD Driven active", "HMD 主導モード動作中", "头显主导模式运行中" },
			{ "Lighthouse Driven active", "Lighthouse 主導モード動作中", "Lighthouse 主导模式运行中" },

			{ "Look left", "左を見てください", "向左看" },
			{ "Look right", "右を見てください", "向右看" },
			{ "Look up", "上を見てください", "向上看" },
			{ "Look down", "下を見てください", "向下看" },
			{ "Look straight ahead", "正面を見てください", "目视正前方" },
			{ "Starting", "開始中", "正在开始" },
			{ "Starting calibration", "キャリブレーションを開始しています", "正在开始校准" },
			{ "Detecting tracker", "トラッカーを検出中", "正在检测追踪器" },
			{ "Move your head around", "頭を動かしてください", "请转动头部" },
			{ "Complete", "完了", "完成" },
			{ "Done", "完了", "完成" },
			{ "Aborted", "中止", "已中止" },
			{ "Calibration failed", "キャリブレーション失敗", "校准失败" },
			{ "Cancel", "キャンセル", "取消" },
			{ "Close", "閉じる", "关闭" },
			{ "Remove", "削除", "删除" },
			{ "Remove Calibration?", "キャリブレーションを削除しますか？", "删除校准？" },
			{ "All offsets will be discarded. You will need to calibrate again.", "すべてのオフセットが破棄されます。再度キャリブレーションが必要です。", "所有偏移将被清除，需要重新校准。" },

			{ "Waiting for SteamVR tracking...", "SteamVR のトラッキングを待機中...", "等待 SteamVR 追踪..." },
			{ "HMD", "HMD", "头显" },
			{ "Left controller", "左コントローラー", "左手柄" },
			{ "Right controller", "右コントローラー", "右手柄" },
			{ "Controller", "コントローラー", "手柄" },
			{ "Head tracker", "ヘッドトラッカー", "头部追踪器" },
			{ "Waist", "腰", "腰部" },
			{ "Chest", "胸", "胸部" },
			{ "Left foot", "左足", "左脚" },
			{ "Right foot", "右足", "右脚" },
			{ "Left knee", "左ひざ", "左膝" },
			{ "Right knee", "右ひざ", "右膝" },
			{ "Left elbow", "左ひじ", "左肘" },
			{ "Right elbow", "右ひじ", "右肘" },
			{ "Left shoulder", "左肩", "左肩" },
			{ "Right shoulder", "右肩", "右肩" },
			{ "Tracker", "トラッカー", "追踪器" },
			{ "Left wrist", "左手首", "左手腕" },
			{ "Right wrist", "右手首", "右手腕" },
			{ "Left ankle", "左足首", "左脚踝" },
			{ "Right ankle", "右足首", "右脚踝" },
			{ "Hand tracker", "ハンドトラッカー", "手部追踪器" },
			{ "Camera", "カメラ", "相机" },
			{ "Keyboard", "キーボード", "键盘" },
			{ "Drag to orbit  \xc2\xb7  Scroll to zoom", "ドラッグで回転  \xc2\xb7  スクロールでズーム", "拖动旋转  \xc2\xb7  滚轮缩放" },

			{ "Turn your basestations on, into standby, or to sleep without a Lighthouse headset. Works with V2 basestations over Bluetooth LE.",
			  "Lighthouse ヘッドセットなしでベースステーションを起動・スタンバイ・スリープにできます。V2 ベースステーション（Bluetooth LE）対応。",
			  "无需 Lighthouse 头显即可唤醒基站、进入待机或休眠。支持 V2 基站（蓝牙 LE）。" },
			{ "Bluetooth LE is not available on this PC.", "この PC では Bluetooth LE を利用できません。", "此电脑不支持蓝牙 LE。" },
			{ "A Bluetooth 4.0+ adapter is required to control basestations.", "ベースステーションの操作には Bluetooth 4.0 以上のアダプターが必要です。", "控制基站需要蓝牙 4.0 及以上的适配器。" },
			{ "Basestation Control", "ベースステーション制御", "基站控制" },
			{ "Lets SpaceSync find your basestations over Bluetooth LE and connect to them. Basestations accept only one connection at a time, so turn this off if another app manages them.",
			  "SpaceSync が Bluetooth LE でベースステーションを検索し、接続できるようにします。ベースステーションは同時に 1 つの接続しか受け付けないため、別のアプリで管理している場合はオフにしてください。",
			  "允许 SpaceSync 通过蓝牙 LE 搜索并连接基站。基站同一时间只接受一个连接，如果由其他应用管理基站，请关闭此选项。" },
			{ "Dynamic Power", "ダイナミックパワー", "动态电源" },
			{ "When enabled, SpaceSync wakes up all basestations as soon as it runs. When SpaceSync gets closed, it puts all basestations into standby.",
			  "有効にすると、SpaceSync の起動時にすべてのベースステーションを起動し、終了時にスタンバイへ切り替えます。",
			  "启用后，SpaceSync 启动时会唤醒所有基站，关闭时会将所有基站切换为待机。" },
			{ "Scanning for basestations...", "ベースステーションを検索中...", "正在扫描基站..." },
			{ "Make sure the stations have power and are within Bluetooth range.", "ベースステーションに電源が入っており、Bluetooth の範囲内にあることを確認してください。", "请确认基站已通电且在蓝牙范围内。" },
			{ "Awake", "起動中", "已唤醒" },
			{ "Standby", "スタンバイ", "待机" },
			{ "Sleeping", "スリープ中", "休眠中" },
			{ "Unknown", "不明", "未知" },
			{ "Wake", "起動", "唤醒" },
			{ "Sleep", "スリープ", "休眠" },
			{ "Refresh", "更新", "刷新" },
			{ "working...", "処理中...", "处理中..." },
			{ "All Basestations", "すべてのベースステーション", "所有基站" },
			{ "Wake all", "すべて起動", "全部唤醒" },
			{ "Standby all", "すべてスタンバイ", "全部待机" },
			{ "Sleep all", "すべてスリープ", "全部休眠" },
			{ "Sleeping or standby stations stop tracking immediately. Standby wakes up faster than sleep; older station firmware only supports sleep.",
			  "スリープ／スタンバイ中はトラッキングが停止します。スタンバイはスリープより復帰が速く、古いファームウェアはスリープのみ対応です。",
			  "休眠或待机的基站会立即停止追踪。待机比休眠唤醒更快；旧固件仅支持休眠。" },
			{ "Setting basestations to standby...", "ベースステーションをスタンバイに設定中...", "正在将基站切换为待机..." },
			{ "Setting basestation \"%s\" to standby...", "ベースステーション「%s」をスタンバイに設定中...", "正在将基站 \"%s\" 切换为待机..." },
			{ "devices tracked", "台のデバイスを追跡中", "个设备已追踪" },
			{ "The window closes when all basestations are in standby.", "すべてのベースステーションがスタンバイになるとウィンドウが閉じます。", "所有基站进入待机后窗口将自动关闭。" },

			{ "NOTE: Changes here take effect instantly, no need to re-calibrate.", "注：ここの変更は即時反映されます。再キャリブレーションは不要です。", "注意：此处的更改立即生效，无需重新校准。" },
			{ "Lighthouse Trackers & Controllers", "Lighthouse トラッカー＆コントローラー", "Lighthouse 追踪器和手柄" },
			{ "Smooths all lighthouse devices (Vive/Tundra trackers, Index controllers) so they show less jittery movement, for example for dancing or full body recordings. Recommended: 25% - smooth movement while keeping latency minimal. The higher the percentage, the more latency you get on fast movement. 0% turns it off.",
			  "すべての Lighthouse デバイス（Vive/Tundra トラッカー、Index コントローラー）の動きを滑らかにします。ダンスやフルボディ収録に最適。推奨は 25%：低遅延のまま滑らかに。数値が高いほど速い動きの遅延が増えます。0% でオフ。",
			  "平滑所有 Lighthouse 设备（Vive/Tundra 追踪器、Index 手柄）的运动，适合跳舞或全身动捕录制。推荐 25%：在保持低延迟的同时平滑运动。百分比越高，快速运动的延迟越大。0% 为关闭。" },
			{ "Latency Compensation", "遅延補正", "延迟补偿" },
			{ "Lighthouse devices reach the headset about 40-60 ms late. SpaceSync predicts where they are right now and learns your movement while you play. 100% removes the delay, 0% turns the prediction off. Lower it if devices overshoot when you stop quickly.",
			  "Lighthouse デバイスの位置はヘッドセットに約 40〜60 ms 遅れて届きます。SpaceSync は今この瞬間の位置を予測し、プレイ中にあなたの動きを学習します。100% で遅延を解消、0% で予測オフ。素早く止めたときにデバイスが行き過ぎる場合は下げてください。",
			  "Lighthouse 设备的位置到达头显时大约延迟 40-60 毫秒。SpaceSync 会预测它们此刻的位置，并在游戏过程中学习你的动作。100% 消除延迟，0% 关闭预测。如果快速停下时设备会冲过头，请调低。" },
			{ "Strength", "強さ", "强度" },
			{ "Headset Tracker", "ヘッドセットトラッカー", "头显追踪器" },
			{ "Smooth headset tracker", "ヘッドセットトラッカーを平滑化", "平滑头显追踪器" },
			{ "Steadies what you see through the headset to reduce shaking (HMD Driven mode only). Adds a tiny bit of delay - if the view feels laggy when you move quickly, adjust the sliders below.",
			  "ヘッドセットの映像の揺れを抑えます（HMD 主導モードのみ）。わずかな遅延が生じます。速く動いたときに遅く感じる場合は下のスライダーを調整してください。",
			  "稳定头显画面、减少抖动（仅头显主导模式）。会带来轻微延迟——若快速移动时感觉拖影，请调整下方滑块。" },
			{ "How steady things look when you are not moving. Lower = calmer image, higher = more responsive (drag right if things feel laggy or floaty).",
			  "静止時の映像の安定度。低いほど滑らか、高いほど反応が速い（遅延やふわつきを感じたら右へ）。",
			  "静止时画面的稳定程度。越低越平稳，越高越灵敏（若感觉延迟或漂浮，请向右拖）。" },
			{ "How quickly the smoothing keeps up when you move fast. Drag right if fast movements feel delayed; drag left if they look shaky.",
			  "速い動きへの追従の速さ。速い動きが遅れて感じるなら右へ、揺れて見えるなら左へ。",
			  "快速移动时平滑的跟随速度。若快速移动感觉延迟请向右拖；若显得抖动请向左拖。" },
			{ "Fine-tunes how the smoothing reacts as your movement speed changes. Most people can leave this alone.",
			  "移動速度の変化に対する反応の微調整。通常は変更不要です。",
			  "微调平滑对移动速度变化的响应。通常无需修改。" },

			{ "NOTE: Most settings below require re-calibration to be applied", "注：以下のほとんどの設定は再キャリブレーション後に反映されます", "注意：以下大多数设置需重新校准后生效" },
			{ "Tracking Method", "トラッキング方式", "追踪方式" },
			{ "HMD Driven", "HMD 主導", "头显主导" },
			{ "Lighthouse Driven", "Lighthouse 主導", "Lighthouse 主导" },
			{ "HMD Driven + No Tracker", "HMD 主導＋トラッカーなし", "头显主导＋无追踪器" },
			{ "Highly Recommended", "特におすすめ", "强烈推荐" },
			{ "Recommended", "おすすめ", "推荐" },
			{ "Not Recommended", "非推奨", "不推荐" },
			{ "Requires a tracker on top of your head.", "頭の上にトラッカーが必要です。", "需要在头顶安装追踪器。" },
			{ "No permanent tracker needed. Drifts over time.", "常設トラッカー不要。時間とともにドリフトします。", "无需常驻追踪器，但会随时间漂移。" },
			{ "The headset owns the tracking space and your lighthouse devices follow it. A tracker on your head keeps them lined up continuously, even when the headset silently re-centres.",
			  "ヘッドセットが空間の基準になり、Lighthouse デバイスがそれに追従します。頭のトラッカーが常時位置合わせを行うため、ヘッドセットが自動リセンターしてもずれません。",
			  "头显作为空间基准，Lighthouse 设备跟随它。头顶追踪器持续对齐，即使头显静默重置中心也不会错位。" },
			{ "The tracker on your head owns the tracking space and the headset follows it. Not recommended on Galaxy XR, Vive Pro or Pico 4, where it can cause jitter.",
			  "頭のトラッカーが空間の基準になり、ヘッドセットが追従します。Galaxy XR / Vive Pro / Pico 4 ではジッターの原因になるため非推奨。",
			  "头顶追踪器作为空间基准，头显跟随它。在 Galaxy XR / Vive Pro / Pico 4 上可能产生抖动，不推荐。" },
			{ "The headset owns the tracking space and your lighthouse devices follow it, but there is no running alignment. You calibrate once by holding a controller or a tracker against your head, then put it back.",
			  "ヘッドセットが空間の基準になりますが、連続アライメントはありません。コントローラーまたはトラッカーを頭に当てて一度だけキャリブレーションし、その後元に戻します。",
			  "头显作为空间基准，但没有持续对齐。将手柄或追踪器贴在头上校准一次，然后放回原处即可。" },
			{ "Fallback to SLAM", "SLAM へフォールバック", "回退到 SLAM" },
			{ "Temporarily uses the headset's own (SLAM) tracking if the head tracker loses line of sight.", "ヘッドトラッカーが見えなくなった間、ヘッドセット自身の SLAM トラッキングを一時使用します。", "当头部追踪器失去视线时，暂时使用头显自身的 SLAM 追踪。" },
			{ "Enable Angular Velocity", "角速度を有効化", "启用角速度" },
			{ "Passes the tracker's angular velocity through to SteamVR. Off by default, it can cause issues with some devices.", "トラッカーの角速度を SteamVR に渡します。既定はオフ。一部デバイスで問題が出る場合があります。", "将追踪器的角速度传递给 SteamVR。默认关闭，某些设备可能出现问题。" },
			{ "Relative Calibration", "相対キャリブレーション", "相对校准" },
			{ "Continuously re-aligns SLAM-tracked devices (controllers etc.) to the calibrated space by comparing the headset's SLAM pose with the tracker-driven pose.",
			  "ヘッドセットの SLAM 姿勢とトラッカー由来の姿勢を比較し、SLAM デバイスを常時再アライメントします。",
			  "通过比较头显 SLAM 姿态与追踪器姿态，持续重新对齐 SLAM 设备（手柄等）。" },
			{ "Hide Head Tracker", "ヘッドトラッカーを非表示", "隐藏头部追踪器" },
			{ "Parks the tracker mounted on your headset far out of the way so games and SteamVR stop treating it as a device in your play space. Alignment is unaffected. Needs HMD Driven with a tracker, and pauses itself while you calibrate.",
			  "頭のトラッカーを遠くへ退避させ、ゲームや SteamVR から見えなくします。アライメントには影響しません。HMD 主導＋トラッカー時のみ。キャリブレーション中は自動的に解除されます。",
			  "将头顶追踪器移到远处，使游戏和 SteamVR 不再把它当作游玩区内的设备。不影响对齐。仅限头显主导＋追踪器模式，校准期间自动暂停。" },
			{ "Stay Aligned", "アライメント維持", "保持对齐" },
			{ "Only for HMD Driven + No Tracker, and it still requires at least one Vive/Tundra hip tracker (SteamVR role \"Waist\"). It reduces drift over time when hiccups occur. It will not completely eliminate drift, but it reduces it drastically.",
			  "HMD 主導＋トラッカーなし専用です。Vive/Tundra の腰トラッカー（SteamVR の役割「Waist」）が少なくとも 1 つ必要です。トラッキングの乱れが起きたときのずれを時間とともに減らします。ずれを完全には無くせませんが、大幅に減らします。",
			  "仅适用于头显主导＋无追踪器模式，并且仍需要至少一个 Vive/Tundra 腰部追踪器（SteamVR 角色“Waist”）。在出现追踪抖动时会随时间减少漂移。它无法完全消除漂移，但能大幅减少。" },
			{ "Inactive: needs HMD Driven + No Tracker.", "無効：HMD 主導＋トラッカーなしが必要です。", "未启用：需要头显主导＋无追踪器模式。" },
			{ "Waiting for calibration.", "キャリブレーション待ち。", "等待校准。" },
			{ "No tracker with SteamVR role Waist found. Only headset recenters and hiccups are handled.",
			  "SteamVR の役割「Waist」のトラッカーが見つかりません。ヘッドセットのリセンターと乱れのみ処理します。",
			  "未找到 SteamVR 角色为 Waist 的追踪器。仅处理头显重新居中和追踪抖动。" },
			{ "Learning the hip tracker. Stand normally for about a minute in total.", "腰トラッカーを学習中です。合計 1 分ほど普通に立ってください。", "正在学习腰部追踪器。请正常站立累计约一分钟。" },
			{ "Re-aligning after a headset pause.", "ヘッドセットを外した後に再アライメント中です。", "摘下头显后正在重新对齐。" },
			{ "Correction", "補正", "修正" },
			{ "recenters", "リセンター", "重新居中" },
			{ "hiccups held", "保持した乱れ", "已保持的抖动" },
			{ "Lock Base Stations", "ベースステーションを固定", "锁定基站" },
			{ "Keeps your base stations where they were when you calibrated. SteamVR sometimes moves base stations by mistake (for example after a tracker loses tracking or the headset wakes from standby), which makes your hands, feet or the whole view jump. Works in all modes. If you really move a base station, just calibrate again.",
			  "キャリブレーション時のベースステーションの位置を保持します。SteamVR は誤ってベースステーションを動かすことがあり（例：トラッカーがトラッキングを失った後や、ヘッドセットがスタンバイから復帰した時）、手や足、または視界全体が飛びます。すべてのモードで動作します。本当にベースステーションを動かした場合は、もう一度キャリブレーションしてください。",
			  "保持基站在校准时的位置。SteamVR 有时会误移基站（例如追踪器丢失追踪后或头显从待机唤醒时），导致手、脚或整个视野跳动。适用于所有模式。如果你确实移动了基站，请重新校准。" },
			{ "Calibrate once to activate the lock.", "固定を有効にするには一度キャリブレーションしてください。", "请校准一次以启用锁定。" },
			{ "Paused while calibrating.", "キャリブレーション中は一時停止しています。", "校准期间已暂停。" },
			{ "Locked base stations", "固定されたベースステーション", "已锁定基站" },
			{ "SteamVR moves held this session", "このセッションで保持した SteamVR の移動", "本次会话已保持的 SteamVR 移动" },
			{ "largest", "最大", "最大" },
			{ "SteamVR currently places your devices away from the calibration by", "SteamVR は現在、デバイスをキャリブレーション時から次の分だけずらしています：", "SteamVR 当前使你的设备偏离校准位置：" },
			{ "Your SteamVR base station layout is inconsistent", "SteamVR のベースステーション配置に矛盾があります", "你的 SteamVR 基站布局不一致" },
			{ "Redo SteamVR Room Setup, then calibrate again.", "SteamVR のルームセットアップをやり直してから、もう一度キャリブレーションしてください。", "请重新进行 SteamVR 房间设置，然后重新校准。" },
			{ "This is held. If you really moved a base station, calibrate again.", "これは保持されています。本当にベースステーションを動かした場合は、もう一度キャリブレーションしてください。", "此偏移已被保持。如果你确实移动了基站，请重新校准。" },
			{ "Disable Voice Help", "音声ガイドを無効化", "关闭语音提示" },
			{ "Disables the voice that tells you how to calibrate during the calibration.", "キャリブレーション中の音声ガイドを無効にします。", "关闭校准过程中的语音提示。" },
			{ "UI Scale", "UI スケール", "界面缩放" },
			{ "Size of text and controls in this window and in the SteamVR dashboard overlay.", "このウィンドウと SteamVR ダッシュボードでの文字とコントロールの大きさ。", "此窗口及 SteamVR 仪表盘中文字和控件的大小。" },
			{ "Calibration Speed", "キャリブレーション速度", "校准速度" },
			{ "Fast", "速い", "快速" },
			{ "Slow", "遅い", "慢速" },
			{ "Very Slow", "とても遅い", "极慢" },
			{ "One pass through the look-around sequence. Quickest option, but small mistakes during calibration show up as inaccuracy.", "見回しシーケンスを 1 周。最速ですが、小さなミスが精度低下につながります。", "环视流程一遍。最快，但校准中的小失误会带来误差。" },
			{ "Recommended. Two passes through the look-around sequence, so the same samples cover more directions.", "推奨。見回しシーケンスを 2 周し、より多くの方向をカバーします。", "推荐。环视流程两遍，样本覆盖更多方向。" },
			{ "Three passes and the most samples. Use this when you want the most precise result.", "3 周・最多サンプル。最高精度が欲しいときに。", "三遍、样本最多。追求最高精度时使用。" },
			{ "Greyed out fields do nothing in HMD Driven mode: the runtime alignment measures yaw and position against your head every frame and undoes those edits within a few seconds. Switch to Lighthouse Driven to use them.",
			  "グレーの項目は HMD 主導モードでは無効です。ランタイムアライメントが毎フレーム補正するため、編集は数秒で打ち消されます。使うには Lighthouse 主導に切り替えてください。",
			  "灰色项在头显主导模式下无效：运行时对齐每帧都会校正，这些修改几秒内就会被抵消。如需使用，请切换到 Lighthouse 主导模式。" },
			{ "Prediction Time", "予測時間", "预测时间" },
			{ "How many frames of prediction SteamVR applies to the tracker. Some wireless solutions may need more prediction to feel smooth.",
			  "SteamVR がトラッカーに適用する予測フレーム数。一部のワイヤレス環境では多めの予測が滑らかに感じられます。",
			  "SteamVR 对追踪器应用的预测帧数。部分无线方案需要更多预测才够流畅。" },
			{ "Step %d of %d", "ステップ %d / %d", "第 %d 步，共 %d 步" },
			{ "The head tracker seems to have moved on the headset. The driver corrected %.1f deg / %.1f cm so far, a fresh calibration is the clean fix.",
			  "ヘッドトラッカーがヘッドセット上でずれた可能性があります。ドライバーはこれまで %.1f 度 / %.1f cm 補正しました。再キャリブレーションが確実な解決策です。",
			  "头部追踪器可能在头显上移位了。驱动已校正 %.1f 度 / %.1f 厘米，重新校准才是彻底的解决办法。" },
			{ "HMD: ", "HMD: ", "头显: " },
			{ " via ", " — ", " — " },
			{ "Yaw", "ヨー", "偏航" },
			{ "Pitch", "ピッチ", "俯仰" },
			{ "Roll", "ロール", "翻滚" },
			{ "Scale", "スケール", "缩放" },
			{ "HMD Scale", "HMD スケール", "头显缩放" },
		};

		Lang g_lang = Lang::English;
		std::unordered_map<std::string, const Entry*> g_index;

		void BuildIndex()
		{
			if (!g_index.empty())
				return;
			for (const Entry& e : kTable)
				g_index[e.en] = &e;
		}
	}

	void SetLanguage(Lang lang)
	{
		g_lang = lang;
	}

	Lang Current()
	{
		return g_lang;
	}

	const char* tr(const char* english)
	{
		if (!english || g_lang == Lang::English || english[0] == '\0')
			return english;
		if (english[0] == '#' && english[1] == '#')
			return english;
		BuildIndex();
		auto it = g_index.find(english);
		if (it == g_index.end())
			return english;
		const Entry* e = it->second;
		const char* out = g_lang == Lang::Japanese ? e->ja : e->zh;
		return out && *out ? out : english;
	}
}
