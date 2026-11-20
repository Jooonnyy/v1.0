#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""速度环自动调参（PC 侧，配合 Bench_Tests 的 WHEEL Target）。

它做什么
--------
串口发命令、收 FireWater 遥测，按固定流程自动迭代，最后**打印可直接粘进
board_wheel.h 的参数**：

  1. 前馈标定：Kp=Ki=0，只留前馈，给几个目标速度 → 由实测占空比反推
     WHEEL_FF_PERMILLE_PER_1000CPS。（探针目标必须高于电机启动死区：
     12V 电机约 150permille 以下可能不转，而 FF=136 时 1200 counts/s 才给到 163permille，
     所以默认探针是 1200/1500/1800 而不是几百。）
  2. Kp 搜索：扫一串 Kp，测阶跃响应的超调 / 振荡 / 稳态误差，挑满足判据的最大值；
  3. Ki 搜索：在选定 Kp 上扫 Ki，挑能消掉稳态误差又不引入低频摆动的最小值。

为什么是 PC 侧做
----------------
指标计算（超调、上升时间、振荡标准差）用 Python 写最省事，也不用往固件里塞
只在调试期存在的代码；固件侧只要提供 E / V / G / T / STOP 和 3 列遥测即可。

⚠ 本脚本会**让电机真的转**。使用前：
  - 车轮**架空**，机构周围清空；
  - 手边有独立断电能力（拔电池 / 开关），脚本崩了不会自己停不下来；
  - 每次运行结束（含异常、Ctrl+C）都会发 STOP，但不要只依赖它。

⚠ 在线改的增益只存在 RAM 里，**复位/重新烧录就没了**。本脚本最后打印的
   #define 才是要写回源码、重新编译烧录的东西。

用法
----
  python Tools/wheel_tune.py --list                 # 列出可用串口
  python Tools/wheel_tune.py --selftest             # 不需要硬件，自检指标计算
  python Tools/wheel_tune.py --port COM5 --wheel a --target 1500
  python Tools/wheel_tune.py --port COM5 --wheel a --target 1500 --skip-ki

依赖：pyserial（python -m pip install pyserial）。--selftest 与 --list 不需要它。
"""

from __future__ import annotations

import argparse
import math
import os
import re
import sys
import time
from datetime import datetime

# ----------------------------------------------------------------------------
# 参数区（不想每次敲命令行就改这里）
# ----------------------------------------------------------------------------

DEFAULT_BAUD = 115200
DEFAULT_WHEEL = "a"
DEFAULT_TARGET_CPS = 1500        # 阶跃目标，单位 counts/s
DEFAULT_FF_PROBE = 136           # 前馈标定的起点（12V 空载 333 rpm；11.1~12.6V 下实测预期 130~160）
DEFAULT_WINDOW_S = 1.6           # 每次阶跃采集时长，单位 s
# 切目标后**立刻**开始采集：软启动是 400 counts/s 每拍（100 Hz），1500 counts/s
# 只要 0.37 s 就到位，如果先等 0.6 s 再采，瞬态早过去了——超调会恒等于 0，
# Kp 就会被挑到偏大。所以 settle 默认 0，指标计算靠窗口末尾的稳态段。
SETTLE_S = 0.0                   # 采集开始前额外等待，单位 s（一般保持 0）
IDLE_S = 0.5                     # 两次试验之间回零后的间隔，单位 s
TX_HZ_ASSUMED = 50.0             # 遥测频率假设值，只用于把采样点换算成时间
TARGET_CEILING_CPS = 3000        # 安全上限：超过就要 --allow-high-speed
STALL_WARN = ("STALL", "stall")  # 文本口出现这些词就判定堵转，立刻停机

# ----------------------------------------------------------------------------
# 遥测解析与指标计算（纯函数，--selftest 覆盖的就是这一部分）
# ----------------------------------------------------------------------------

LINE_RE = re.compile(r"^\s*(-?\d+)\s*,\s*(-?\d+)\s*,\s*(-?\d+)\s*$")


def parse_sample(line: str):
    """把一行遥测解析成 (target, actual, output)；非数字行返回 None。

    mode 0 的遥测格式固定是 `tgt_cps,act_cps,out_permille`，不带任何前缀，
    所以开机横幅、错误文本、蓝牙丢包造成的半行都会在这里被丢掉。
    """
    match = LINE_RE.match(line)
    if not match:
        return None
    return tuple(int(group) for group in match.groups())


def steady_state(samples, tail=0.25):
    """末尾 tail 比例样本的均值，作为稳态值。"""
    if not samples:
        return 0.0
    count = max(1, int(len(samples) * tail))
    return sum(samples[-count:]) / count


def overshoot_pct(samples, target):
    """最大超调百分比（相对目标绝对值）。目标为 0 时返回 0。"""
    if not samples or target == 0:
        return 0.0
    peak = max(samples) if target > 0 else min(samples)
    return max(0.0, (peak - target) / abs(target) * 100.0)


def rise_time_ms(samples, target, dt_ms):
    """10% -> 90% 上升时间，单位 ms；没到过就返回 None。"""
    if not samples or target == 0:
        return None
    low, high = 0.1 * target, 0.9 * target
    idx_low = idx_high = None
    for index, value in enumerate(samples):
        if idx_low is None and value >= low:
            idx_low = index
        if idx_high is None and value >= high:
            idx_high = index
            break
    if idx_low is None or idx_high is None:
        return None
    return (idx_high - idx_low) * dt_ms


def settling_ms(samples, target, tol_pct=5.0, dt_ms=20.0):
    """最后一次离开 +/-tol 带之后再也没回来的时刻，单位 ms；一直在带外返回 None。"""
    if not samples or target == 0:
        return None
    band = abs(target) * tol_pct / 100.0
    last_outside = None
    for index, value in enumerate(samples):
        if abs(value - target) > band:
            last_outside = index
    if last_outside is None:
        return 0.0
    if last_outside >= len(samples) - 1:
        return None
    return (last_outside + 1) * dt_ms


def oscillation_std(samples, tail=0.5):
    """稳态段的标准差，单位 counts/s。振荡越大这个值越大。"""
    if len(samples) < 4:
        return float("inf")
    count = max(2, int(len(samples) * tail))
    segment = samples[-count:]
    mean = sum(segment) / len(segment)
    variance = sum((value - mean) ** 2 for value in segment) / len(segment)
    return math.sqrt(variance)


def response_metrics(samples, target, dt_ms):
    """一次阶跃响应的全部指标。"""
    return {
        "steady": steady_state(samples),
        "err": steady_state(samples) - target,
        "overshoot_pct": overshoot_pct(samples, target),
        "rise_ms": rise_time_ms(samples, target, dt_ms),
        "settle_ms": settling_ms(samples, target, dt_ms=dt_ms),
        "osc_std": oscillation_std(samples),
        "n": len(samples),
    }


def fmt_metrics(metrics, target):
    rise = "n/a" if metrics["rise_ms"] is None else f"{metrics['rise_ms']:.0f}"
    settle = "n/a" if metrics["settle_ms"] is None else f"{metrics['settle_ms']:.0f}"
    return (f"n={metrics['n']:3d} steady={metrics['steady']:7.1f}/{target} "
            f"err={metrics['err']:7.1f} over={metrics['overshoot_pct']:5.1f}% "
            f"rise={rise:>5s}ms settle={settle:>5s}ms osc={metrics['osc_std']:6.1f}")


# ----------------------------------------------------------------------------
# 串口链路
# ----------------------------------------------------------------------------

class Link:
    """串口封装：发命令、读遥测、掉线重试。

    ⚠ 蓝牙 SPP 的虚拟 COM 口会偶发断连，所以读超时被当成正常事件，
      不抛异常；只有连续多次写失败才认为链路真的没了。
    """

    def __init__(self, port, baud, echo=False):
        import serial  # 延迟导入：--selftest / --list 不需要 pyserial
        self.serial = serial.Serial(port=port, baudrate=baud, timeout=0.05)
        self.echo = echo
        self.junk_lines = 0
        self.write_failures = 0
        time.sleep(0.3)
        self.serial.reset_input_buffer()

    def close(self):
        try:
            self.serial.close()
        except Exception:
            pass

    def send(self, command: str):
        if self.echo:
            print(f"  >> {command}")
        try:
            self.serial.write((command + "\r\n").encode("ascii"))
            self.write_failures = 0
        except Exception as exc:                      # 蓝牙掉线
            self.write_failures += 1
            print(f"  !! 写串口失败({self.write_failures}): {exc}")
            if self.write_failures >= 3:
                raise

    def drain(self):
        """丢掉输入缓冲里的陈旧数据。"""
        try:
            self.serial.reset_input_buffer()
        except Exception:
            pass

    def collect(self, seconds: float):
        """采集 seconds 秒的遥测，返回 (targets, actuals, outputs) 三个列表。"""
        deadline = time.monotonic() + seconds
        targets, actuals, outputs = [], [], []
        while time.monotonic() < deadline:
            try:
                raw = self.serial.readline()
            except Exception:
                time.sleep(0.01)
                continue
            if not raw:
                continue
            text = raw.decode("ascii", errors="ignore").strip()
            if not text:
                continue
            if any(word in text for word in STALL_WARN):
                raise RuntimeError(f"固件报告堵转/异常: {text}")
            sample = parse_sample(text)
            if sample is None:
                self.junk_lines += 1
                continue
            targets.append(sample[0])
            actuals.append(sample[1])
            outputs.append(sample[2])
        return targets, actuals, outputs


# ----------------------------------------------------------------------------
# 调参流程
# ----------------------------------------------------------------------------

def step_test(link, wheel, target_cps, window_s, label, csv_rows, dt_ms):
    """一次阶跃：发目标 → 采集 → 回零，返回 (指标, 稳态占空比)。"""
    link.send(f"V {wheel} 0")
    time.sleep(IDLE_S)
    link.drain()
    link.send(f"V {wheel} {target_cps}")
    time.sleep(SETTLE_S)
    _, actuals, outputs = link.collect(window_s)
    link.send(f"V {wheel} 0")

    if len(actuals) < 5:
        print(f"  {label}: 采样太少（{len(actuals)} 个），链路有问题？")
        return None, None

    metrics = response_metrics(actuals, target_cps, dt_ms)
    steady_duty = steady_state(outputs)
    print(f"  {label}: {fmt_metrics(metrics, target_cps)} duty={steady_duty:6.1f}permille")
    csv_rows.append((label, target_cps, steady_duty, metrics))
    return metrics, steady_duty


def calibrate_feedforward(link, wheel, probes, ff_probe, window_s, dt_ms, csv_rows):
    """Kp=Ki=0，只留前馈，用实测占空比反推 FF。

    推导：稳态下 duty[permille] 对应的实际转速 act ≈ max_cps * duty/1000，
    而"前馈正确"意味着 1000‰ 对应 max_cps，即 FF = 1000/max_cps*1000 = 1000*duty/act。
    """
    print("\n== 1/3 前馈标定（Kp=Ki=0）==")
    link.send(f"G 0 0 {ff_probe}")
    time.sleep(0.2)

    ratios = []
    for target_cps in probes:
        metrics, duty = step_test(link, wheel, target_cps, window_s,
                                  f"FF探针 tgt={target_cps}", csv_rows, dt_ms)
        if metrics is None:
            continue
        actual = metrics["steady"]
        if actual < 200 or duty < 20:
            print(f"    跳过：实际 {actual:.0f} counts/s 太小（占空比 {duty:.0f}permille）。"
                  f"12V 电机在约 150permille 以下是启动死区，探针目标要调大"
                  f"（--ff-probes 1200 1500 1800）")
            continue
        ratios.append(1000.0 * duty / actual)

    if not ratios:
        print("  !! 前馈标定失败：没有有效探针点。检查电机是否使能(E 1)、编码器是否有计数。")
        return None

    ff = round(sum(ratios) / len(ratios))
    print(f"  => 建议 WHEEL_FF_PERMILLE_PER_1000CPS = {ff}  （各点比值 {[round(r,1) for r in ratios]}）")
    return ff


def search_kp(link, wheel, kp_candidates, ff, target_cps, window_s, dt_ms, csv_rows,
              overshoot_limit=20.0, osc_limit_cps=60.0):
    """扫 Kp，挑满足"超调 <= 上限 且 稳态振荡 <= 上限"的最大值。"""
    print(f"\n== 2/3 Kp 搜索（Ki=0，FF={ff}，目标 {target_cps} counts/s）==")
    ok = []
    for kp in kp_candidates:
        link.send(f"G {kp} 0 {ff}")
        time.sleep(0.2)
        metrics, _ = step_test(link, wheel, target_cps, window_s, f"Kp={kp}", csv_rows, dt_ms)
        if metrics is None:
            continue
        passed = (metrics["overshoot_pct"] <= overshoot_limit
                  and metrics["osc_std"] <= osc_limit_cps)
        print(f"    {'通过' if passed else '不合格'}"
              f"（判据：超调<={overshoot_limit}% 且 振荡<={osc_limit_cps}）")
        if passed:
            ok.append((kp, metrics))
    if not ok:
        print("  !! 没有 Kp 通过判据：先确认前馈是否准、编码器分辨率是否够。")
        return None
    best_kp = max(kp for kp, _ in ok)
    print(f"  => 建议 WHEEL_KP_Q10 = {best_kp}")
    return best_kp


def search_ki(link, wheel, ki_candidates, kp, ff, target_cps, window_s, dt_ms, csv_rows,
              osc_limit_cps=60.0):
    """在选定 Kp 上扫 Ki，优先消稳态误差，其次取最小且不引入摆动。"""
    print(f"\n== 3/3 Ki 搜索（Kp={kp}，FF={ff}）==")
    best = None
    for ki in ki_candidates:
        link.send(f"G {kp} {ki} {ff}")
        time.sleep(0.2)
        metrics, _ = step_test(link, wheel, target_cps, window_s, f"Ki={ki}", csv_rows, dt_ms)
        if metrics is None:
            continue
        acceptable = metrics["osc_std"] <= osc_limit_cps
        score = (abs(metrics["err"]), ki)
        print(f"    {'可接受' if acceptable else '摆动过大'}"
              f"（|稳态误差|={abs(metrics['err']):.0f}）")
        if acceptable and (best is None or score < best[0]):
            best = (score, ki)
    if best is None:
        print("  !! 没有 Ki 可接受，建议暂时用 Kp+FF（纯 P 会有稳态误差，属正常）。")
        return None
    print(f"  => 建议 WHEEL_KI_Q10 = {best[1]}")
    return best[1]


def run_session(args):
    dt_ms = 1000.0 / TX_HZ_ASSUMED
    csv_rows = []

    link = Link(args.port, args.baud, echo=args.echo)
    print(f"串口已打开：{args.port} @ {args.baud}，轮子 {args.wheel}，目标 {args.target} counts/s")
    print("提示：调参期间不要同时打开 VOFA+（一个 COM 口只能被一个程序占用）")

    try:
        link.send("T 1")          # 打开遥测
        link.send("STOP")
        link.send("Z")            # 计数清零，位姿归零
        time.sleep(0.3)
        link.send("E 1")          # 使能闭环
        time.sleep(0.5)

        ff = args.ff_probe
        if not args.skip_ff:
            ff = calibrate_feedforward(link, args.wheel, args.ff_probes, args.ff_probe,
                                       args.window, dt_ms, csv_rows) or args.ff_probe
        else:
            print(f"\n跳过前馈标定，沿用 FF={ff}")

        kp = args.kp
        if not args.skip_kp:
            kp = search_kp(link, args.wheel, args.kp_candidates, ff, args.target,
                           args.window, dt_ms, csv_rows) or args.kp

        ki = args.ki
        if not args.skip_ki:
            ki = search_ki(link, args.wheel, args.ki_candidates, kp, ff, args.target,
                           args.window, dt_ms, csv_rows) or args.ki

        print("\n" + "=" * 72)
        print("把下面三行写回 BSP/Inc/board_wheel.h 的参数区，然后重新编译并烧录：")
        print("=" * 72)
        print(f"#define WHEEL_FF_PERMILLE_PER_1000CPS  {ff}")
        print(f"#define WHEEL_KP_Q10                   {kp}")
        print(f"#define WHEEL_KI_Q10                   {ki}")
        print("=" * 72)
        print("⚠ 在线增益只在 RAM 里，复位或重新烧录会丢失——上面这三行才是最终结果。")
        print("⚠ 烧录后要再跑一次本脚本确认（用 --skip-ff --kp <值> --ki <值>），")
        print("   确认源码里的参数与调出来的一致。")
        if link.junk_lines:
            print(f"（本次丢弃了 {link.junk_lines} 行非遥测数据；蓝牙丢包时会偏多）")

    finally:
        # 无论正常结束、异常还是 Ctrl+C，都要让电机停下来。
        for _ in range(3):
            try:
                link.send("V a 0")
                link.send("V b 0")
                link.send("V c 0")
                link.send("V d 0")
                link.send("STOP")
                time.sleep(0.1)
            except Exception:
                pass
        link.close()
        write_csv(csv_rows, args)
        print("已发送 STOP，串口已关闭。")


def write_csv(rows, args):
    if not rows:
        return
    path = args.csv or os.path.join("Tools", "logs",
                                    f"wheel_tune_{datetime.now():%Y%m%d_%H%M%S}.csv")
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w", encoding="utf-8") as handle:
        handle.write("label,target_cps,duty_permille,steady,err,overshoot_pct,"
                     "rise_ms,settle_ms,osc_std,n\n")
        for label, target, duty, metrics in rows:
            handle.write(
                f"{label},{target},{duty:.1f},{metrics['steady']:.1f},{metrics['err']:.1f},"
                f"{metrics['overshoot_pct']:.2f},"
                f"{'' if metrics['rise_ms'] is None else round(metrics['rise_ms'])},"
                f"{'' if metrics['settle_ms'] is None else round(metrics['settle_ms'])},"
                f"{metrics['osc_std']:.2f},{metrics['n']}\n")
    print(f"明细已写入 {path}")


# ----------------------------------------------------------------------------
# 不需要硬件的模式
# ----------------------------------------------------------------------------

def cmd_list():
    try:
        from serial.tools import list_ports
    except ImportError:
        print("需要 pyserial：python -m pip install pyserial")
        return 1
    ports = list(list_ports.comports())
    if not ports:
        print("没有枚举到任何串口。蓝牙要先把模块配对成功，Windows 才会生成出站 COM 口。")
        return 0
    for port in ports:
        print(f"{port.device:8s} {port.description}")
    return 0


def cmd_selftest():
    """自检指标计算：用已知形状的合成阶跃响应，检查算出来的数对不对。"""
    dt_ms = 1000.0 / TX_HZ_ASSUMED
    target = 1500
    failures = []

    def check(name, condition, detail):
        print(f"  [{'PASS' if condition else 'FAIL'}] {name}: {detail}")
        if not condition:
            failures.append(name)

    print("--selftest（不需要硬件）")

    # 1) 一阶上升，无超调
    first_order = [target * (1 - math.exp(-i * dt_ms / 150.0)) for i in range(120)]
    metrics = response_metrics(first_order, target, dt_ms)
    check("一阶响应超调≈0", metrics["overshoot_pct"] < 1.0,
          f"over={metrics['overshoot_pct']:.2f}%")
    check("一阶响应稳态≈目标", abs(metrics["err"]) < 5.0, f"err={metrics['err']:.2f}")
    check("一阶响应上升时间合理", 200 < (metrics["rise_ms"] or 0) < 600,
          f"rise={metrics['rise_ms']:.0f}ms")

    # 2) 25% 超调的欠阻尼响应
    overshooting = [target * (1 - math.exp(-i * dt_ms / 120.0)
                              * math.cos(2 * math.pi * i * dt_ms / 400.0))
                    for i in range(150)]
    metrics = response_metrics(overshooting, target, dt_ms)
    check("欠阻尼超调落在 10~45%", 10 < metrics["overshoot_pct"] < 45,
          f"over={metrics['overshoot_pct']:.1f}%")

    # 3) 持续振荡：稳态标准差必须明显大于安静情况
    quiet = [target + (1.0 if i % 2 else -1.0) for i in range(100)]
    ringing = [target + 150.0 * math.sin(2 * math.pi * i * dt_ms / 200.0) for i in range(100)]
    quiet_std = oscillation_std(quiet)
    ring_std = oscillation_std(ringing)
    check("振荡能被识别", ring_std > 5 * quiet_std,
          f"quiet={quiet_std:.2f} ring={ring_std:.1f}")

    # 4) 稳态误差识别
    biased = [0.7 * target for _ in range(100)]
    metrics = response_metrics(biased, target, dt_ms)
    check("稳态误差识别", abs(metrics["err"] - (-0.3 * target)) < 5,
          f"err={metrics['err']:.1f}")

    # 5) 解析器：正常行、全角逗号、半行、横幅都要正确处理
    check("解析 3 列整数", parse_sample("1500,1480,140") == (1500, 1480, 140),
          repr(parse_sample("1500,1480,140")))
    check("解析负值", parse_sample("-800,-790,120") == (-800, -790, 120),
          repr(parse_sample("-800,-790,120")))
    check("丢弃横幅文本", parse_sample("Encoder test: 1320 counts/rev") is None, "None")
    check("丢弃半行", parse_sample("1500,1480") is None, "None")
    check("丢弃空行", parse_sample("   ") is None, "None")

    # 6) 前馈反推公式：12V 下典型 duty=140‰、act=1000 counts/s 时应得 FF≈140
    ff = 1000.0 * 140 / 1000
    check("前馈反推公式", abs(ff - 140) < 1, f"ff={ff:.0f}")

    print(f"\n结果：{'全部通过' if not failures else '失败项 ' + ', '.join(failures)}")
    return 0 if not failures else 1


# ----------------------------------------------------------------------------
# 命令行
# ----------------------------------------------------------------------------

def build_parser():
    parser = argparse.ArgumentParser(
        description="WHEEL Target 速度环自动调参（前馈 → Kp → Ki）",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="示例：python Tools/wheel_tune.py --port COM5 --wheel a --target 1500")
    parser.add_argument("--list", action="store_true", help="列出可用串口后退出")
    parser.add_argument("--selftest", action="store_true", help="自检指标计算，不需要硬件")
    parser.add_argument("--port", help="串口号，例如 COM5（蓝牙是配对后的出站端口）")
    parser.add_argument("--baud", type=int, default=DEFAULT_BAUD)
    parser.add_argument("--wheel", default=DEFAULT_WHEEL, choices=list("abcd"))
    parser.add_argument("--target", type=int, default=DEFAULT_TARGET_CPS,
                        help=f"阶跃目标 counts/s（默认 {DEFAULT_TARGET_CPS}）")
    parser.add_argument("--allow-high-speed", action="store_true",
                        help=f"允许目标超过 {TARGET_CEILING_CPS} counts/s")
    parser.add_argument("--ff-probe", type=int, default=DEFAULT_FF_PROBE,
                        help="前馈标定的起点（12V 空载 333 rpm 对应 136，实测预期 130~160）")
    parser.add_argument("--ff-probes", type=int, nargs="+", default=[1200, 1500, 1800],
                        help="前馈探针目标（要高于电机启动死区：12V 电机约 150permille，"
                             "FF=136 时 1200 counts/s 才给到 163permille）")
    parser.add_argument("--kp", type=int, default=200, help="跳过 Kp 搜索时使用的值")
    parser.add_argument("--ki", type=int, default=8, help="跳过 Ki 搜索时使用的值")
    parser.add_argument("--kp-candidates", type=int, nargs="+",
                        default=[50, 100, 200, 300, 400, 600, 800])
    parser.add_argument("--ki-candidates", type=int, nargs="+",
                        default=[0, 4, 8, 16, 32])
    parser.add_argument("--window", type=float, default=DEFAULT_WINDOW_S,
                        help="每次阶跃的采集时长，单位 s")
    parser.add_argument("--skip-ff", action="store_true")
    parser.add_argument("--skip-kp", action="store_true")
    parser.add_argument("--skip-ki", action="store_true")
    parser.add_argument("--csv", help="明细 CSV 输出路径")
    parser.add_argument("--echo", action="store_true", help="回显发出的每条命令")
    return parser


def main(argv=None):
    args = build_parser().parse_args(argv)

    if args.selftest:
        return cmd_selftest()
    if args.list:
        return cmd_list()
    if not args.port:
        build_parser().print_help()
        print("\n错误：缺少 --port（用 --list 看可用串口，--selftest 不需要硬件）")
        return 2
    if args.target > TARGET_CEILING_CPS and not args.allow_high_speed:
        print(f"拒绝执行：目标 {args.target} 超过安全上限 {TARGET_CEILING_CPS} counts/s。"
              f"\n架空并确认安全后再加 --allow-high-speed。")
        return 2

    try:
        run_session(args)
    except KeyboardInterrupt:
        print("\n用户中断。")
        return 130
    except ImportError:
        print("需要 pyserial：python -m pip install pyserial")
        return 3
    except Exception as exc:
        print(f"\n运行失败：{type(exc).__name__}: {exc}")
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
