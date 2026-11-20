#!/bin/bash
# 物料相机成像参数一键锁定。
#
# 为什么需要这个：v4l2 的 -c 设置存在驱动里，USB 一插拔就全部回到默认值。
# 默认的 auto_exposure=3（光圈优先）+ white_balance_automatic=1 会让相机自己
# 决定曝光和白平衡 —— 补光灯一亮它就把画面压暗来抵消，标出来的 HSV 和肉眼
# 看到的完全不是一回事。所以每次插拔或重启后，标定/测试之前都要跑一次。
#
# 用法：
#   ./cam_setup.sh              # 用默认曝光 60
#   ./cam_setup.sh 45           # 指定曝光时间（单位 100us，45 = 4.5ms）
#   ./cam_setup.sh --check      # 只读当前值，不改
#   ./cam_setup.sh --list       # 列出这台相机支持的全部控件
#
# 判据：--check 输出里应当是
#   auto_exposure: 1 (Manual Mode)
#   white_balance_automatic: 0
#   且 exposure_time_absolute 那一行**不带** inactive 标记。

DEV=${DEV:-/dev/cam_target}
WB_TEMP=${WB_TEMP:-4600}

if [ "$1" = "--list" ]; then
    v4l2-ctl -d "$DEV" --list-ctrls
    exit 0
fi

if [ "$1" = "--check" ]; then
    v4l2-ctl -d "$DEV" --get-ctrl \
        auto_exposure,exposure_time_absolute,white_balance_automatic,\
white_balance_temperature,gain,exposure_dynamic_framerate
    exit 0
fi

EXPOSURE=${1:-60}

set -e
v4l2-ctl -d "$DEV" \
    -c auto_exposure=1 \
    -c exposure_time_absolute="$EXPOSURE" \
    -c white_balance_automatic=0 \
    -c white_balance_temperature="$WB_TEMP" \
    -c gain=0 \
    -c exposure_dynamic_framerate=0

echo "已锁定：exposure=$EXPOSURE (${EXPOSURE}00us)，白平衡 ${WB_TEMP}K，增益 0"
echo ""
v4l2-ctl -d "$DEV" --get-ctrl \
    auto_exposure,exposure_time_absolute,white_balance_automatic
echo ""
echo "确认 auto_exposure 是 1 (Manual Mode)、white_balance_automatic 是 0。"
echo "然后跑 vision_node.py --preview 看亮度：物料颜色饱和但不发白才对；"
echo "太暗就调大曝光值，过曝（发白）就调小。"
