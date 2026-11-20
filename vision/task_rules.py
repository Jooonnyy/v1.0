# -*- coding: utf-8 -*-
"""任务码解析：4 组三位数 -> 两批物料的抓取顺序与放置圆环。

格式（与 STM32 端 task_code_valid() 一致）：
    15 个字符，4 组三位数，组间 '+', 每位 1..6
    第 1 组：第一批 3 个物料的颜色/搬运顺序
    第 2 组：第一批 3 个物料在粗加工区/暂存区的放置圆环号
    第 3 组：第二批 3 个物料的颜色/搬运顺序
    第 4 组：第二批 3 个物料在粗加工区的放置圆环号

颜色：1 红、2 黄、3 蓝、4 绿、5 黑、6 浅蓝。

例：156+123+516+231
    第一批：红(1)→环1、黑(5)→环2、浅蓝(6)→环3
    第二批：黑(5)→环2、红(1)→环3、浅蓝(6)→环1
"""

COLOR_NAMES = {1: "红", 2: "黄", 3: "蓝", 4: "绿", 5: "黑", 6: "浅蓝"}

# 第二组的 3 个位置分别属于哪个区：规则里写的是“粗加工区和暂存区”，
# 但没给三位与两个区的对应关系，这里先留空，等现场确认后填。
# 例：("粗加工区", "暂存区", "暂存区")
BATCH1_AREAS = (None, None, None)
# 第四组明确是粗加工区。
BATCH2_AREAS = ("粗加工区", "粗加工区", "粗加工区")


def validate(code):
    """与固件 task_code_valid() 相同的格式检查。"""
    if not code or len(code) != 15:
        return False
    for index, char in enumerate(code):
        if index in (3, 7, 11):
            if char != "+":
                return False
        elif char not in "123456":
            return False
    return True


def _items(colors, rings, areas):
    items = []
    for seq, (color, ring, area) in enumerate(zip(colors, rings, areas), 1):
        items.append({
            "seq": seq,
            "color": color,
            "name": COLOR_NAMES.get(color, "?"),
            "ring": ring,
            "area": area,
        })
    return items


def parse(code):
    """解析任务码；格式不对抛 ValueError。返回 dict（可直接写 JSON）。"""
    code = (code or "").strip()
    if not validate(code):
        raise ValueError("任务码格式应为 4 组三位数、每位 1..6，例如 156+123+516+231")
    groups = code.split("+")
    batch1_colors = [int(ch) for ch in groups[0]]
    batch1_rings = [int(ch) for ch in groups[1]]
    batch2_colors = [int(ch) for ch in groups[2]]
    batch2_rings = [int(ch) for ch in groups[3]]
    return {
        "raw": code,
        "batch1": _items(batch1_colors, batch1_rings, BATCH1_AREAS),
        "batch2": _items(batch2_colors, batch2_rings, BATCH2_AREAS),
        "grab_order": [item["color"] for item in
                       _items(batch1_colors, batch1_rings, BATCH1_AREAS)] +
                      [item["color"] for item in
                       _items(batch2_colors, batch2_rings, BATCH2_AREAS)],
    }


def describe(plan):
    """给人看的两行。"""
    def line(batch):
        parts = []
        for item in batch:
            if item["area"]:
                parts.append(f"{item['name']}({item['color']})→{item['area']} {item['ring']}号环")
            else:
                parts.append(f"{item['name']}({item['color']})→{item['ring']}号环")
        return "；".join(parts)

    return (f"第一批：{line(plan['batch1'])}\n"
            f"第二批：{line(plan['batch2'])}")


if __name__ == "__main__":
    import json
    import sys

    text = sys.argv[1] if len(sys.argv) > 1 else "156+123+516+231"
    plan = parse(text)
    print(describe(plan))
    print(json.dumps(plan, ensure_ascii=False, indent=2))
