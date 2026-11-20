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
			const char* ko;
		};

		const Entry kTable[] = {
			{ "SteamVR Running", "SteamVR 起動中", "SteamVR 运行中", "SteamVR 실행중" },
			{ "SteamVR Not Running", "SteamVR 未起動", "SteamVR 未运行", "SteamVR 실행되지 않음" },

			{ "Calibration", "キャリブレーション", "校准", "보정" },
			{ "Tracking Preview", "トラッキングプレビュー", "追踪预览", "추적 미리보기" },
			{ "Smoothing", "スムージング", "平滑", "스무딩" },
			{ "Basestations", "ベースステーション", "基站", "베이스스테이션" },
			{ "Settings", "設定", "设置", "설정" },

			{ "Calibrate", "キャリブレーション", "开始校准", "보정" },
			{ "Calibrated", "キャリブレーション済み", "已校准", "보정됨" },
			{ "Click to start", "クリックして開始", "点击开始", "눌러서 시작" },
			{ "Click to calibrate again", "クリックで再キャリブレーション", "点击重新校准", "눌러서 재보정" },
			{ "Start SteamVR to calibrate", "キャリブレーションには SteamVR を起動してください", "请先启动 SteamVR 再校准", "SteamVR을 실행하여 보정" },
			{ "not detected", "未検出", "未检测到", "감지되지 않음" },
			{ "Edit Calibration", "キャリブレーションを編集", "编辑校准", "보정값 수정" },
			{ "Remove Calibration", "キャリブレーションを削除", "删除校准", "보정값 삭제" },
			{ "Save Profile", "プロファイルを保存", "保存配置", "프로파일 저장" },
			{ "Back", "戻る", "返回", "뒤로가기" },
			{ "No calibration yet", "キャリブレーション未実施", "尚未校准", "아직 보정되지 않음" },
			{ "Click the circle to calibrate", "円をクリックしてキャリブレーション", "点击圆圈开始校准", "원을 클릭하여 보정" },
			{ "SpaceSync driver not loaded", "SpaceSync ドライバー未読み込み", "SpaceSync 驱动未加载", "SpaceSync 드라이버가 로드되지 않음" },
			{ "Restart SteamVR and enable the SpaceSync add-on", "SteamVR を再起動し、SpaceSync アドオンを有効にしてください", "请重启 SteamVR 并启用 SpaceSync 插件", "SteamVR을 재시작하고 SpaceSync 플러그인을 활성화하세요" },
			{ "Headset tracker not connected", "ヘッドセットトラッカー未接続", "头显追踪器未连接", "헤드셋 트래커가 연결되지 않음" },
			{ "Calibrated without a head tracker", "ヘッドトラッカーなしでキャリブレーション済み", "在没有头部追踪器的情况下校准", "머리 트래커없이 보정됨" },
			{ "Calibrate again with the tracker on your head", "頭にトラッカーを付けて再キャリブレーションしてください", "请把追踪器戴在头上重新校准", "머리에 트래커를 장착 후 재보정하세요" },
			{ "Not active yet without a head tracker. Calibrate once so SpaceSync can measure the delay.",
			  "ヘッドトラッカーなしではまだ有効ではありません。遅延を測定するため、一度キャリブレーションしてください。",
			  "没有头部追踪器时尚未启用。请校准一次，让 SpaceSync 测量延迟。", "머리 트래커 없이는 이 기능을 사용할 수 없습니다. SpaceSync에서 지연 시간을 측정할 수 있도록 한 번 보정하세요." },
			{ "HMD Driven, one-time calibration", "HMD 主導・ワンタイムキャリブレーション", "头显主导・一次性校准", "HMD 기준, 1회 보정" },
			{ "no running alignment", "連続アライメントなし", "无持续对齐", "실시간 정렬 없음" },
			{ "Override disabled", "オーバーライド無効", "覆盖已禁用", "오버라이드 꺼짐" },
			{ "HMD tracking system changed?", "HMD のトラッキング方式が変わった可能性", "头显追踪系统可能已更改", "HMD 트래킹 방식이 바뀌었나요?" },
			{ "HMD Driven active", "HMD 主導モード動作中", "头显主导模式运行中", "HMD 기준 모드 작동 중" },
			{ "Lighthouse Driven active", "Lighthouse 主導モード動作中", "Lighthouse 主导模式运行中", "Lighthouse 기준 모드 작동 중" },

			{ "Look left", "左を見てください", "向左看", "왼쪽을 보세요" },
			{ "Look right", "右を見てください", "向右看", "오른쪽을 보세요" },
			{ "Look up", "上を見てください", "向上看", "위를 보세요" },
			{ "Look down", "下を見てください", "向下看", "아래를 보세요" },
			{ "Look straight ahead", "正面を見てください", "目视正前方", "정면을 보세요" },
			{ "Starting", "開始中", "正在开始", "시작 중" },
			{ "Starting calibration", "キャリブレーションを開始しています", "正在开始校准", "보정을 시작하는 중" },
			{ "Detecting tracker", "トラッカーを検出中", "正在检测追踪器", "트래커 감지 중" },
			{ "Move your head around", "頭を動かしてください", "请转动头部", "고개를 돌려 주세요" },
			{ "Complete", "完了", "完成", "완료" },
			{ "Done", "完了", "完成", "완료" },
			{ "Aborted", "中止", "已中止", "중단됨" },
			{ "Calibration failed", "キャリブレーション失敗", "校准失败", "보정 실패" },
			{ "Cancel", "キャンセル", "取消", "취소" },
			{ "Close", "閉じる", "关闭", "닫기" },
			{ "Remove", "削除", "删除", "삭제" },
			{ "Remove Calibration?", "キャリブレーションを削除しますか？", "删除校准？", "보정값을 삭제할까요?" },
			{ "All offsets will be discarded. You will need to calibrate again.", "すべてのオフセットが破棄されます。再度キャリブレーションが必要です。", "所有偏移将被清除，需要重新校准。", "모든 오프셋이 삭제됩니다. 다시 보정해야 합니다." },

			{ "Waiting for SteamVR tracking...", "SteamVR のトラッキングを待機中...", "等待 SteamVR 追踪...", "SteamVR 트래킹을 기다리는 중..." },
			{ "HMD", "HMD", "头显", "HMD" },
			{ "Left controller", "左コントローラー", "左手柄", "왼쪽 컨트롤러" },
			{ "Right controller", "右コントローラー", "右手柄", "오른쪽 컨트롤러" },
			{ "Controller", "コントローラー", "手柄", "컨트롤러" },
			{ "Head tracker", "ヘッドトラッカー", "头部追踪器", "머리 트래커" },
			{ "Waist", "腰", "腰部", "허리" },
			{ "Chest", "胸", "胸部", "가슴" },
			{ "Left foot", "左足", "左脚", "왼발" },
			{ "Right foot", "右足", "右脚", "오른발" },
			{ "Left knee", "左ひざ", "左膝", "왼쪽 무릎" },
			{ "Right knee", "右ひざ", "右膝", "오른쪽 무릎" },
			{ "Left elbow", "左ひじ", "左肘", "왼쪽 팔꿈치" },
			{ "Right elbow", "右ひじ", "右肘", "오른쪽 팔꿈치" },
			{ "Left shoulder", "左肩", "左肩", "왼쪽 어깨" },
			{ "Right shoulder", "右肩", "右肩", "오른쪽 어깨" },
			{ "Tracker", "トラッカー", "追踪器", "트래커" },
			{ "Left wrist", "左手首", "左手腕", "왼쪽 손목" },
			{ "Right wrist", "右手首", "右手腕", "오른쪽 손목" },
			{ "Left ankle", "左足首", "左脚踝", "왼쪽 발목" },
			{ "Right ankle", "右足首", "右脚踝", "오른쪽 발목" },
			{ "Hand tracker", "ハンドトラッカー", "手部追踪器", "핸드 트래커" },
			{ "Camera", "カメラ", "相机", "카메라" },
			{ "Keyboard", "キーボード", "键盘", "키보드" },
			{ "Drag to orbit  \xc2\xb7  Scroll to zoom", "ドラッグで回転  \xc2\xb7  スクロールでズーム", "拖动旋转  \xc2\xb7  滚轮缩放", "드래그로 회전  \xc2\xb7  스크롤로 확대/축소" },

			{ "Turn your basestations on, into standby, or to sleep without a Lighthouse headset. Works with V2 basestations over Bluetooth LE.",
			  "Lighthouse ヘッドセットなしでベースステーションを起動・スタンバイ・スリープにできます。V2 ベースステーション（Bluetooth LE）対応。",
			  "无需 Lighthouse 头显即可唤醒基站、进入待机或休眠。支持 V2 基站（蓝牙 LE）。", "Lighthouse 헤드셋 없이도 베이스스테이션을 켜거나 대기, 절전 모드로 전환할 수 있습니다. 블루투스 LE를 통해 V2 베이스스테이션을 지원합니다." },
			{ "Bluetooth LE is not available on this PC.", "この PC では Bluetooth LE を利用できません。", "此电脑不支持蓝牙 LE。", "이 PC에서는 블루투스 LE를 사용할 수 없습니다." },
			{ "A Bluetooth 4.0+ adapter is required to control basestations.", "ベースステーションの操作には Bluetooth 4.0 以上のアダプターが必要です。", "控制基站需要蓝牙 4.0 及以上的适配器。", "베이스스테이션을 제어하려면 블루투스 4.0 이상 어댑터가 필요합니다." },
			{ "Dynamic Power", "ダイナミックパワー", "动态电源", "다이나믹 파워" },
			{ "When enabled, SpaceSync wakes up all basestations as soon as it runs. When SpaceSync gets closed, it puts all basestations into standby or sleep (see Dynamic Power Mode).",
			  "有効にすると、SpaceSync の起動時にすべてのベースステーションを起動し、終了時にスタンバイまたはスリープへ切り替えます（ダイナミックパワーモードを参照）。",
			  "启用后，SpaceSync 启动时会唤醒所有基站，关闭时会将所有基站切换为待机或休眠（见动态电源模式）。", "활성화하면 SpaceSync 실행 시 모든 베이스스테이션을 깨우고, SpaceSync를 종료할 때 모든 베이스스테이션을 대기 또는 절전 모드로 전환합니다(다이나믹 파워 모드 참조)." },
			{ "Basestation Management", "ベースステーション管理", "基站管理", "베이스스테이션 관리" },
			{ "SpaceSync connects to your basestations over Bluetooth to show and switch their power state. Turn this off if you manage your basestations with another tool.",
			  "SpaceSync は Bluetooth でベースステーションに接続し、電源状態の表示と切り替えを行います。別のツールでベースステーションを管理している場合はオフにしてください。",
			  "SpaceSync 通过蓝牙连接基站，用于显示和切换其电源状态。如果你使用其他工具管理基站，请关闭此项。", "SpaceSync가 블루투스로 베이스스테이션에 연결해 전원 상태를 표시하고 전환합니다. 다른 도구로 베이스스테이션을 관리한다면 이 기능을 끄세요." },
			{ "Basestation management is off. SpaceSync does not connect to your basestations over Bluetooth.",
			  "ベースステーション管理はオフです。SpaceSync は Bluetooth でベースステーションに接続しません。",
			  "基站管理已关闭。SpaceSync 不会通过蓝牙连接你的基站。", "베이스스테이션 관리가 꺼져 있습니다. SpaceSync가 블루투스로 베이스스테이션에 연결하지 않습니다." },
			{ "Dynamic Power Mode", "ダイナミックパワーモード", "动态电源模式", "다이나믹 파워 모드" },
			{ "What the basestations do when SpaceSync closes. Standby wakes up faster, sleep uses less power.",
			  "SpaceSync 終了時のベースステーションの状態です。スタンバイは復帰が速く、スリープは消費電力が少なくなります。",
			  "SpaceSync 关闭时基站的状态。待机唤醒更快，休眠更省电。", "SpaceSync가 종료될 때 베이스스테이션이 취할 동작입니다. 대기 모드는 더 빨리 깨어나고, 절전 모드는 전력을 더 적게 사용합니다." },
			{ "Setting basestations to sleep...", "ベースステーションをスリープに設定中...", "正在将基站切换为休眠...", "베이스스테이션을 절전 모드로 전환하는 중..." },
			{ "Setting basestation \"%s\" to sleep...", "ベースステーション「%s」をスリープに設定中...", "正在将基站 \"%s\" 切换为休眠...", "베이스스테이션 \"%s\"을(를) 절전 모드로 전환하는 중..." },
			{ "The window closes when all basestations are asleep.", "すべてのベースステーションがスリープになるとウィンドウが閉じます。", "所有基站进入休眠后窗口将自动关闭。", "모든 베이스스테이션이 절전 모드가 되면 창이 닫힙니다." },
			{ "Scanning for basestations...", "ベースステーションを検索中...", "正在扫描基站...", "베이스스테이션을 검색하는 중..." },
			{ "Make sure the stations have power and are within Bluetooth range.", "ベースステーションに電源が入っており、Bluetooth の範囲内にあることを確認してください。", "请确认基站已通电且在蓝牙范围内。", "베이스스테이션의 전원이 켜져 있고 블루투스 범위 안에 있는지 확인하세요." },
			{ "Awake", "起動中", "已唤醒", "작동 중" },
			{ "Standby", "スタンバイ", "待机", "대기" },
			{ "Sleeping", "スリープ中", "休眠中", "절전 중" },
			{ "Unknown", "不明", "未知", "알 수 없음" },
			{ "Wake", "起動", "唤醒", "깨우기" },
			{ "Sleep", "スリープ", "休眠", "절전" },
			{ "Refresh", "更新", "刷新", "새로고침" },
			{ "working...", "処理中...", "处理中...", "처리 중..." },
			{ "All Basestations", "すべてのベースステーション", "所有基站", "모든 베이스스테이션" },
			{ "Wake all", "すべて起動", "全部唤醒", "모두 깨우기" },
			{ "Standby all", "すべてスタンバイ", "全部待机", "모두 대기" },
			{ "Sleep all", "すべてスリープ", "全部休眠", "모두 절전" },
			{ "Sleeping or standby stations stop tracking immediately. Standby wakes up faster than sleep; older station firmware only supports sleep.",
			  "スリープ／スタンバイ中はトラッキングが停止します。スタンバイはスリープより復帰が速く、古いファームウェアはスリープのみ対応です。",
			  "休眠或待机的基站会立即停止追踪。待机比休眠唤醒更快；旧固件仅支持休眠。", "절전 또는 대기 중인 베이스스테이션은 즉시 트래킹을 멈춥니다. 대기 모드는 절전보다 빨리 깨어나며, 구형 펌웨어는 절전 모드만 지원합니다." },
			{ "Setting basestations to standby...", "ベースステーションをスタンバイに設定中...", "正在将基站切换为待机...", "베이스스테이션을 대기 모드로 전환하는 중..." },
			{ "Setting basestation \"%s\" to standby...", "ベースステーション「%s」をスタンバイに設定中...", "正在将基站 \"%s\" 切换为待机...", "베이스스테이션 \"%s\"을(를) 대기 모드로 전환하는 중..." },
			{ "devices tracked", "台のデバイスを追跡中", "个设备已追踪", "개 기기 추적 중" },
			{ "The window closes when all basestations are in standby.", "すべてのベースステーションがスタンバイになるとウィンドウが閉じます。", "所有基站进入待机后窗口将自动关闭。", "모든 베이스스테이션이 대기 모드가 되면 창이 닫힙니다." },

			{ "NOTE: Changes here take effect instantly, no need to re-calibrate.", "注：ここの変更は即時反映されます。再キャリブレーションは不要です。", "注意：此处的更改立即生效，无需重新校准。", "참고: 여기서 변경한 내용은 즉시 적용되며 다시 보정할 필요가 없습니다." },
			{ "Lighthouse Trackers & Controllers", "Lighthouse トラッカー＆コントローラー", "Lighthouse 追踪器和手柄", "Lighthouse 트래커 및 컨트롤러" },
			{ "Smooths all lighthouse devices (Vive/Tundra trackers, Index controllers) so they show less jittery movement, for example for dancing or full body recordings. Recommended: 25% - smooth movement while keeping latency minimal. The higher the percentage, the more latency you get on fast movement. 0% turns it off.",
			  "すべての Lighthouse デバイス（Vive/Tundra トラッカー、Index コントローラー）の動きを滑らかにします。ダンスやフルボディ収録に最適。推奨は 25%：低遅延のまま滑らかに。数値が高いほど速い動きの遅延が増えます。0% でオフ。",
			  "平滑所有 Lighthouse 设备（Vive/Tundra 追踪器、Index 手柄）的运动，适合跳舞或全身动捕录制。推荐 25%：在保持低延迟的同时平滑运动。百分比越高，快速运动的延迟越大。0% 为关闭。", "모든 Lighthouse 기기(Vive/Tundra 트래커, Index 컨트롤러)의 움직임을 부드럽게 만들어 떨림을 줄입니다. 춤이나 풀바디 녹화에 적합합니다. 권장값은 25%로, 지연을 최소로 유지하면서 움직임을 부드럽게 해 줍니다. 수치가 높을수록 빠른 움직임에서 지연이 커집니다. 0%는 꺼짐입니다." },
			{ "Latency Compensation", "遅延補正", "延迟补偿", "지연 보상" },
			{ "Lighthouse devices reach the headset about 40-60 ms late. SpaceSync predicts where they are right now and learns your movement while you play. 100% removes the delay, 0% turns the prediction off. Lower it if devices overshoot when you stop quickly.",
			  "Lighthouse デバイスの位置はヘッドセットに約 40〜60 ms 遅れて届きます。SpaceSync は今この瞬間の位置を予測し、プレイ中にあなたの動きを学習します。100% で遅延を解消、0% で予測オフ。素早く止めたときにデバイスが行き過ぎる場合は下げてください。",
			  "Lighthouse 设备的位置到达头显时大约延迟 40-60 毫秒。SpaceSync 会预测它们此刻的位置，并在游戏过程中学习你的动作。100% 消除延迟，0% 关闭预测。如果快速停下时设备会冲过头，请调低。", "Lighthouse 기기의 위치는 헤드셋에 약 40~60ms 늦게 도착합니다. SpaceSync는 기기의 현재 위치를 예측하고, 플레이하는 동안 사용자의 움직임을 학습합니다. 100%는 지연을 완전히 없애고, 0%는 예측을 끕니다. 갑자기 멈출 때 기기가 지나치게 튀어나가면 값을 낮추세요." },
			{ "Strength", "強さ", "强度", "강도" },
			{ "Headset Tracker", "ヘッドセットトラッカー", "头显追踪器", "헤드셋 트래커" },
			{ "Smooth headset tracker", "ヘッドセットトラッカーを平滑化", "平滑头显追踪器", "헤드셋 트래커 스무딩" },
			{ "Steadies what you see through the headset to reduce shaking (HMD Driven mode only). Adds a tiny bit of delay - if the view feels laggy when you move quickly, adjust the sliders below.",
			  "ヘッドセットの映像の揺れを抑えます（HMD 主導モードのみ）。わずかな遅延が生じます。速く動いたときに遅く感じる場合は下のスライダーを調整してください。",
			  "稳定头显画面、减少抖动（仅头显主导模式）。会带来轻微延迟——若快速移动时感觉拖影，请调整下方滑块。", "헤드셋으로 보이는 화면을 안정시켜 흔들림을 줄입니다(HMD 기준 모드 전용). 약간의 지연이 생기므로, 빠르게 움직일 때 화면이 끌리는 느낌이면 아래 슬라이더를 조정하세요." },
			{ "How steady things look when you are not moving. Lower = calmer image, higher = more responsive (drag right if things feel laggy or floaty).",
			  "静止時の映像の安定度。低いほど滑らか、高いほど反応が速い（遅延やふわつきを感じたら右へ）。",
			  "静止时画面的稳定程度。越低越平稳，越高越灵敏（若感觉延迟或漂浮，请向右拖）。", "움직이지 않을 때 화면이 얼마나 안정적으로 보일지 정합니다. 낮을수록 차분하고, 높을수록 반응이 빠릅니다(지연되거나 둥둥 뜨는 느낌이 들면 오른쪽으로 올리세요)." },
			{ "How quickly the smoothing keeps up when you move fast. Drag right if fast movements feel delayed; drag left if they look shaky.",
			  "速い動きへの追従の速さ。速い動きが遅れて感じるなら右へ、揺れて見えるなら左へ。",
			  "快速移动时平滑的跟随速度。若快速移动感觉延迟请向右拖；若显得抖动请向左拖。", "빠르게 움직일 때 스무딩이 얼마나 빨리 따라오는지 정합니다. 빠른 움직임이 늦게 느껴지면 오른쪽으로, 떨려 보이면 왼쪽으로 옮기세요." },
			{ "Fine-tunes how the smoothing reacts as your movement speed changes. Most people can leave this alone.",
			  "移動速度の変化に対する反応の微調整。通常は変更不要です。",
			  "微调平滑对移动速度变化的响应。通常无需修改。", "움직임 속도가 바뀔 때 스무딩이 반응하는 방식을 미세 조정합니다. 대부분은 그대로 두셔도 됩니다." },

			{ "NOTE: Most settings below require re-calibration to be applied", "注：以下のほとんどの設定は再キャリブレーション後に反映されます", "注意：以下大多数设置需重新校准后生效", "참고: 아래 설정 대부분은 다시 보정해야 적용됩니다" },
			{ "Tracking Method", "トラッキング方式", "追踪方式", "트래킹 방식" },
			{ "HMD Driven", "HMD 主導", "头显主导", "HMD 기준" },
			{ "Lighthouse Driven", "Lighthouse 主導", "Lighthouse 主导", "Lighthouse 기준" },
			{ "HMD Driven + No Tracker", "HMD 主導＋トラッカーなし", "头显主导＋无追踪器", "HMD 기준 + 트래커 없음" },
			{ "Highly Recommended", "特におすすめ", "强烈推荐", "강력 추천" },
			{ "Recommended", "おすすめ", "推荐", "추천" },
			{ "Not Recommended", "非推奨", "不推荐", "비추천" },
			{ "Requires a tracker on top of your head.", "頭の上にトラッカーが必要です。", "需要在头顶安装追踪器。", "머리 위에 트래커가 필요합니다." },
			{ "No permanent tracker needed. Drifts over time.", "常設トラッカー不要。時間とともにドリフトします。", "无需常驻追踪器，但会随时间漂移。", "상시 트래커는 필요 없지만 시간이 지나면 드리프트가 생깁니다." },
			{ "The headset owns the tracking space and your lighthouse devices follow it. A tracker on your head keeps them lined up continuously, even when the headset silently re-centres.",
			  "ヘッドセットが空間の基準になり、Lighthouse デバイスがそれに追従します。頭のトラッカーが常時位置合わせを行うため、ヘッドセットが自動リセンターしてもずれません。",
			  "头显作为空间基准，Lighthouse 设备跟随它。头顶追踪器持续对齐，即使头显静默重置中心也不会错位。", "헤드셋이 트래킹 공간의 기준이 되고 Lighthouse 기기가 그것을 따라갑니다. 머리의 트래커가 계속 위치를 맞춰 주므로, 헤드셋이 조용히 중심을 다시 잡아도 어긋나지 않습니다." },
			{ "The tracker on your head owns the tracking space and the headset follows it. Not recommended on Galaxy XR, Vive Pro or Pico 4, where it can cause jitter.",
			  "頭のトラッカーが空間の基準になり、ヘッドセットが追従します。Galaxy XR / Vive Pro / Pico 4 ではジッターの原因になるため非推奨。",
			  "头顶追踪器作为空间基准，头显跟随它。在 Galaxy XR / Vive Pro / Pico 4 上可能产生抖动，不推荐。", "머리의 트래커가 트래킹 공간의 기준이 되고 헤드셋이 그것을 따라갑니다. Galaxy XR, Vive Pro, Pico 4에서는 떨림이 생길 수 있어 권장하지 않습니다." },
			{ "The headset owns the tracking space and your lighthouse devices follow it, but there is no running alignment. You calibrate once by holding a controller or a tracker against your head, then put it back.",
			  "ヘッドセットが空間の基準になりますが、連続アライメントはありません。コントローラーまたはトラッカーを頭に当てて一度だけキャリブレーションし、その後元に戻します。",
			  "头显作为空间基准，但没有持续对齐。将手柄或追踪器贴在头上校准一次，然后放回原处即可。", "헤드셋이 트래킹 공간의 기준이 되지만 실시간 정렬은 없습니다. 컨트롤러나 트래커를 머리에 대고 한 번만 보정한 뒤 원래 자리에 돌려놓으세요." },
			{ "Fallback to SLAM", "SLAM へフォールバック", "回退到 SLAM", "SLAM으로 대체" },
			{ "Temporarily uses the headset's own (SLAM) tracking if the head tracker loses line of sight.", "ヘッドトラッカーが見えなくなった間、ヘッドセット自身の SLAM トラッキングを一時使用します。", "当头部追踪器失去视线时，暂时使用头显自身的 SLAM 追踪。", "머리 트래커가 시야에서 사라지면 헤드셋 자체의 SLAM 트래킹을 임시로 사용합니다." },
			{ "Enable Angular Velocity", "角速度を有効化", "启用角速度", "각속도 활성화" },
			{ "Passes the tracker's angular velocity through to SteamVR. Off by default, it can cause issues with some devices.", "トラッカーの角速度を SteamVR に渡します。既定はオフ。一部デバイスで問題が出る場合があります。", "将追踪器的角速度传递给 SteamVR。默认关闭，某些设备可能出现问题。", "트래커의 각속도를 SteamVR로 전달합니다. 기본값은 꺼짐이며, 일부 기기에서 문제가 생길 수 있습니다." },
			{ "Relative Calibration", "相対キャリブレーション", "相对校准", "상대 보정" },
			{ "Continuously re-aligns SLAM-tracked devices (controllers etc.) to the calibrated space by comparing the headset's SLAM pose with the tracker-driven pose.",
			  "ヘッドセットの SLAM 姿勢とトラッカー由来の姿勢を比較し、SLAM デバイスを常時再アライメントします。",
			  "通过比较头显 SLAM 姿态与追踪器姿态，持续重新对齐 SLAM 设备（手柄等）。", "헤드셋의 SLAM 자세와 트래커 기반 자세를 비교해, SLAM으로 추적되는 기기(컨트롤러 등)를 보정된 공간에 계속 다시 맞춥니다." },
			{ "Hide Head Tracker", "ヘッドトラッカーを非表示", "隐藏头部追踪器", "머리 트래커 숨기기" },
			{ "Parks the tracker mounted on your headset far out of the way so games and SteamVR stop treating it as a device in your play space. Alignment is unaffected. Needs HMD Driven with a tracker, and pauses itself while you calibrate.",
			  "頭のトラッカーを遠くへ退避させ、ゲームや SteamVR から見えなくします。アライメントには影響しません。HMD 主導＋トラッカー時のみ。キャリブレーション中は自動的に解除されます。",
			  "将头顶追踪器移到远处，使游戏和 SteamVR 不再把它当作游玩区内的设备。不影响对齐。仅限头显主导＋追踪器模式，校准期间自动暂停。", "헤드셋에 장착한 트래커를 멀리 치워 두어 게임과 SteamVR이 플레이 공간의 기기로 인식하지 않게 합니다. 정렬에는 영향이 없습니다. 트래커를 쓰는 HMD 기준 모드에서만 동작하며, 보정 중에는 자동으로 일시 정지됩니다." },
			{ "Stay Aligned", "アライメント維持", "保持对齐", "정렬 유지" },
			{ "Only for HMD Driven + No Tracker, and it still requires at least one Vive/Tundra hip tracker (SteamVR role \"Waist\"). It reduces drift over time when hiccups occur. It will not completely eliminate drift, but it reduces it drastically.",
			  "HMD 主導＋トラッカーなし専用です。Vive/Tundra の腰トラッカー（SteamVR の役割「Waist」）が少なくとも 1 つ必要です。トラッキングの乱れが起きたときのずれを時間とともに減らします。ずれを完全には無くせませんが、大幅に減らします。",
			  "仅适用于头显主导＋无追踪器模式，并且仍需要至少一个 Vive/Tundra 腰部追踪器（SteamVR 角色“Waist”）。在出现追踪抖动时会随时间减少漂移。它无法完全消除漂移，但能大幅减少。", "HMD 기준 + 트래커 없음 모드 전용이며, Vive/Tundra 허리 트래커(SteamVR 역할 \"Waist\")가 최소 하나 필요합니다. 트래킹이 순간적으로 튈 때 시간이 지나며 생기는 드리프트를 줄여 줍니다. 드리프트를 완전히 없애지는 못하지만 크게 줄여 줍니다." },
			{ "Inactive: needs HMD Driven + No Tracker.", "無効：HMD 主導＋トラッカーなしが必要です。", "未启用：需要头显主导＋无追踪器模式。", "비활성: HMD 기준 + 트래커 없음 모드가 필요합니다." },
			{ "Waiting for calibration.", "キャリブレーション待ち。", "等待校准。", "보정을 기다리는 중입니다." },
			{ "No tracker with SteamVR role Waist found. Only headset recenters and hiccups are handled.",
			  "SteamVR の役割「Waist」のトラッカーが見つかりません。ヘッドセットのリセンターと乱れのみ処理します。",
			  "未找到 SteamVR 角色为 Waist 的追踪器。仅处理头显重新居中和追踪抖动。", "SteamVR 역할이 Waist인 트래커를 찾을 수 없습니다. 헤드셋 중심 재설정과 트래킹 튐만 처리합니다." },
			{ "Learning the hip tracker. Stand normally for about a minute in total.", "腰トラッカーを学習中です。合計 1 分ほど普通に立ってください。", "正在学习腰部追踪器。请正常站立累计约一分钟。", "허리 트래커를 학습하는 중입니다. 합계 1분 정도 평소처럼 서 계세요." },
			{ "Re-aligning after a headset pause.", "ヘッドセットを外した後に再アライメント中です。", "摘下头显后正在重新对齐。", "헤드셋을 벗었다 다시 쓴 뒤 재정렬하는 중입니다." },
			{ "Correction", "補正", "修正", "교정량" },
			{ "recenters", "リセンター", "重新居中", "중심 재설정" },
			{ "hiccups held", "保持した乱れ", "已保持的抖动", "유지한 튐" },
			{ "Lock Base Stations", "ベースステーションを固定", "锁定基站", "베이스스테이션 고정" },
			{ "Keeps your base stations where they were when you calibrated. SteamVR sometimes moves base stations by mistake (for example after a tracker loses tracking or the headset wakes from standby), which makes your hands, feet or the whole view jump. Works in all modes. If you really move a base station, just calibrate again.",
			  "キャリブレーション時のベースステーションの位置を保持します。SteamVR は誤ってベースステーションを動かすことがあり（例：トラッカーがトラッキングを失った後や、ヘッドセットがスタンバイから復帰した時）、手や足、または視界全体が飛びます。すべてのモードで動作します。本当にベースステーションを動かした場合は、もう一度キャリブレーションしてください。",
			  "保持基站在校准时的位置。SteamVR 有时会误移基站（例如追踪器丢失追踪后或头显从待机唤醒时），导致手、脚或整个视野跳动。适用于所有模式。如果你确实移动了基站，请重新校准。", "보정했을 때의 베이스스테이션 위치를 그대로 유지합니다. SteamVR은 가끔 베이스스테이션을 잘못 옮기는데(예: 트래커가 트래킹을 잃은 뒤나 헤드셋이 대기 모드에서 깨어날 때), 그러면 손과 발, 또는 시야 전체가 튑니다. 모든 모드에서 동작합니다. 베이스스테이션을 실제로 옮겼다면 다시 보정하세요." },
			{ "Calibrate once to activate the lock.", "固定を有効にするには一度キャリブレーションしてください。", "请校准一次以启用锁定。", "고정을 활성화하려면 한 번 보정하세요." },
			{ "Paused while calibrating.", "キャリブレーション中は一時停止しています。", "校准期间已暂停。", "보정 중에는 일시 정지됩니다." },
			{ "Locked base stations", "固定されたベースステーション", "已锁定基站", "고정된 베이스스테이션" },
			{ "SteamVR moves held this session", "このセッションで保持した SteamVR の移動", "本次会话已保持的 SteamVR 移动", "이번 세션에서 유지한 SteamVR 이동" },
			{ "largest", "最大", "最大", "최대" },
			{ "SteamVR currently places your devices away from the calibration by", "SteamVR は現在、デバイスをキャリブレーション時から次の分だけずらしています：", "SteamVR 当前使你的设备偏离校准位置：", "SteamVR이 현재 기기를 보정 위치에서 다음만큼 벗어나게 배치하고 있습니다:" },
			{ "Your SteamVR base station layout is inconsistent", "SteamVR のベースステーション配置に矛盾があります", "你的 SteamVR 基站布局不一致", "SteamVR 베이스스테이션 배치가 일관되지 않습니다" },
			{ "Redo SteamVR Room Setup, then calibrate again.", "SteamVR のルームセットアップをやり直してから、もう一度キャリブレーションしてください。", "请重新进行 SteamVR 房间设置，然后重新校准。", "SteamVR 룸 설정을 다시 진행한 뒤 다시 보정하세요." },
			{ "This is held. If you really moved a base station, calibrate again.", "これは保持されています。本当にベースステーションを動かした場合は、もう一度キャリブレーションしてください。", "此偏移已被保持。如果你确实移动了基站，请重新校准。", "이 어긋남은 유지되고 있습니다. 베이스스테이션을 실제로 옮겼다면 다시 보정하세요." },
			{ "Disable Voice Help", "音声ガイドを無効化", "关闭语音提示", "음성 안내 끄기" },
			{ "Disables the voice that tells you how to calibrate during the calibration.", "キャリブレーション中の音声ガイドを無効にします。", "关闭校准过程中的语音提示。", "보정하는 동안 방법을 알려 주는 음성 안내를 끕니다." },
			{ "UI Scale", "UI スケール", "界面缩放", "UI 크기" },
			{ "Size of text and controls in this window and in the SteamVR dashboard overlay.", "このウィンドウと SteamVR ダッシュボードでの文字とコントロールの大きさ。", "此窗口及 SteamVR 仪表盘中文字和控件的大小。", "이 창과 SteamVR 대시보드 오버레이에 표시되는 글자와 컨트롤의 크기입니다." },
			{ "Calibration Speed", "キャリブレーション速度", "校准速度", "보정 속도" },
			{ "Fast", "速い", "快速", "빠름" },
			{ "Slow", "遅い", "慢速", "느림" },
			{ "Very Slow", "とても遅い", "极慢", "매우 느림" },
			{ "One pass through the look-around sequence. Quickest option, but small mistakes during calibration show up as inaccuracy.", "見回しシーケンスを 1 周。最速ですが、小さなミスが精度低下につながります。", "环视流程一遍。最快，但校准中的小失误会带来误差。", "둘러보기 동작을 한 번 진행합니다. 가장 빠르지만 보정 중의 작은 실수가 정확도 저하로 이어집니다." },
			{ "Recommended. Two passes through the look-around sequence, so the same samples cover more directions.", "推奨。見回しシーケンスを 2 周し、より多くの方向をカバーします。", "推荐。环视流程两遍，样本覆盖更多方向。", "권장. 둘러보기 동작을 두 번 진행해 같은 샘플로 더 많은 방향을 다룹니다." },
			{ "Three passes and the most samples. Use this when you want the most precise result.", "3 周・最多サンプル。最高精度が欲しいときに。", "三遍、样本最多。追求最高精度时使用。", "세 번 진행하며 샘플이 가장 많습니다. 가장 정밀한 결과가 필요할 때 사용하세요." },
			{ "Greyed out fields do nothing in HMD Driven mode: the runtime alignment measures yaw and position against your head every frame and undoes those edits within a few seconds. Switch to Lighthouse Driven to use them.",
			  "グレーの項目は HMD 主導モードでは無効です。ランタイムアライメントが毎フレーム補正するため、編集は数秒で打ち消されます。使うには Lighthouse 主導に切り替えてください。",
			  "灰色项在头显主导模式下无效：运行时对齐每帧都会校正，这些修改几秒内就会被抵消。如需使用，请切换到 Lighthouse 主导模式。", "회색으로 표시된 항목은 HMD 기준 모드에서는 효과가 없습니다. 런타임 정렬이 매 프레임 머리를 기준으로 요와 위치를 측정해 몇 초 안에 수정 내용을 되돌리기 때문입니다. 사용하려면 Lighthouse 기준 모드로 전환하세요." },
			{ "Prediction Time", "予測時間", "预测时间", "예측 시간" },
			{ "How many frames of prediction SteamVR applies to the tracker. Some wireless solutions may need more prediction to feel smooth.",
			  "SteamVR がトラッカーに適用する予測フレーム数。一部のワイヤレス環境では多めの予測が滑らかに感じられます。",
			  "SteamVR 对追踪器应用的预测帧数。部分无线方案需要更多预测才够流畅。", "SteamVR이 트래커에 적용하는 예측 프레임 수입니다. 일부 무선 환경에서는 예측이 더 많아야 부드럽게 느껴질 수 있습니다." },
			{ "Step %d of %d", "ステップ %d / %d", "第 %d 步，共 %d 步", "%d / %d 단계" },
			{ "The head tracker seems to have moved on the headset. The driver corrected %.1f deg / %.1f cm so far, a fresh calibration is the clean fix.",
			  "ヘッドトラッカーがヘッドセット上でずれた可能性があります。ドライバーはこれまで %.1f 度 / %.1f cm 補正しました。再キャリブレーションが確実な解決策です。",
			  "头部追踪器可能在头显上移位了。驱动已校正 %.1f 度 / %.1f 厘米，重新校准才是彻底的解决办法。", "머리 트래커가 헤드셋에서 움직인 것 같습니다. 드라이버가 지금까지 %.1f도 / %.1fcm를 보정했으며, 새로 보정하는 것이 가장 깔끔한 해결책입니다." },
			{ "HMD: ", "HMD: ", "头显: ", "HMD: " },
			{ " via ", " — ", " — ", " — " },
			{ "Yaw", "ヨー", "偏航", "요" },
			{ "Pitch", "ピッチ", "俯仰", "피치" },
			{ "Roll", "ロール", "翻滚", "롤" },
			{ "Scale", "スケール", "缩放", "스케일" },
			{ "HMD Scale", "HMD スケール", "头显缩放", "HMD 스케일" },
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
		const char* out = nullptr;
		switch (g_lang)
		{
		case Lang::Japanese: out = e->ja; break;
		case Lang::Chinese: out = e->zh; break;
		case Lang::Korean: out = e->ko; break;
		default: break;
		}
		return out && *out ? out : english;
	}
}
