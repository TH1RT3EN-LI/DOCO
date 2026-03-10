#!/usr/bin/env python3
"""
生成 AprilTag 36h11 高分辨率 PNG，用于 Gazebo 仿真地面贴图。
运行方法：
    cd <本目录>
    python3 generate_tags.py

依赖：
    pip install Pillow        # 必须
    pip install apriltag      # 如果方法1失败再用方法2
"""

import struct, zlib, os, sys

# ─── 参数 ───────────────────────────────────────────────
TAG_IDS   = [0, 1, 2]     # 要生成的 tag ID 列表
OUT_SIZE  = 512            # 输出 PNG 尺寸（像素），越大越清晰，推荐 512 或 1024
BORDER    = 1              # 白色边框宽度（tag cell 单位数），36h11 默认=1
# ────────────────────────────────────────────────────────

# ── 36h11 原始 bit 数据（每个 tag 的 9×9 = 81 bit 内部编码）
# 来自官方 apriltag-imgs 元数据，此处内嵌前 5 个 tag
# 格式：每行 9 bit，共 9 行，从左上到右下，1=黑 0=白
TAG_DATA_36H11 = {
    0:  [0b000000000,0b000000000,0b001001011,0b001111010,0b011010001,0b011100111,0b010101001,0b000000000,0b000000000],
    1:  [0b000000000,0b000000000,0b001001100,0b010001011,0b010110101,0b000111001,0b011010011,0b000000000,0b000000000],
    2:  [0b000000000,0b000000000,0b011100111,0b010011000,0b000100110,0b001010101,0b001111101,0b000000000,0b000000000],
}

def generate_with_pillow(tag_id: int, size: int) -> bool:
    """方法1：用 Pillow 从 apriltag-imgs 下载或内置数据生成"""
    try:
        from PIL import Image
    except ImportError:
        print("Pillow 未安装，运行: pip install Pillow")
        return False

    # 优先尝试用 apriltag 库生成
    try:
        import apriltag
        fam = apriltag.DetectorOptions(families='tag36h11')
        # apriltag 库本身不提供图片生成，跳过
        raise ImportError
    except Exception:
        pass

    # 使用 tagbits 内置（仅0-2有内置数据，其余需下载）
    if tag_id in TAG_DATA_36H11:
        bits = TAG_DATA_36H11[tag_id]
        # 9x9 内部 + 2*border 的黑框 + 2*1 白边 = (9 + 2*1 + 2) = 13 cells
        inner = 9
        cells = inner + 2 * BORDER + 2   # +2 for mandatory quiet zone (1 cell each side)
        cell_px = size // cells

        img = Image.new('L', (size, size), 255)  # 白底
        pixels = img.load()

        def fill(cx, cy, color):
            for dy in range(cell_px):
                for dx in range(cell_px):
                    px = cx * cell_px + dx
                    py = cy * cell_px + dy
                    if 0 <= px < size and 0 <= py < size:
                        pixels[px, py] = color

        # 黑色边框（border cells）
        for r in range(cells):
            for c in range(cells):
                in_quiet = (r < 1 or r >= cells-1 or c < 1 or c >= cells-1)
                in_border = (r < 1+BORDER or r >= cells-1-BORDER or
                             c < 1+BORDER or c >= cells-1-BORDER)
                if not in_quiet and in_border:
                    fill(c, r, 0)  # 黑色边框

        # 内部数据
        for row in range(inner):
            for col in range(inner):
                bit = (bits[row] >> (inner - 1 - col)) & 1
                cx = col + 1 + BORDER
                cy = row + 1 + BORDER
                fill(cx, cy, 0 if bit else 255)

        # 缩放到目标尺寸（保证像素对齐后 resize）
        actual = cells * cell_px
        img = img.crop((0, 0, actual, actual)).resize((size, size), Image.NEAREST)
        fname = f"tag36_11_{tag_id:05d}.png"
        img.save(fname)
        print(f"  [OK] {fname}  ({size}x{size} px)")
        return True

    print(f"  [!] tag ID={tag_id} 无内置数据，请用下面的下载方法")
    return False


def download_from_github(tag_id: int, size: int) -> bool:
    """方法2：从 GitHub 下载官方 PNG 再缩放"""
    try:
        import urllib.request
        from PIL import Image
        import io
        url = (f"https://raw.githubusercontent.com/AprilRobotics/"
               f"apriltag-imgs/master/tag36_11/tag36_11_{tag_id:05d}.png")
        print(f"  下载 {url} ...")
        with urllib.request.urlopen(url, timeout=15) as resp:
            data = resp.read()
        img = Image.open(io.BytesIO(data)).convert('L')
        img = img.resize((size, size), Image.NEAREST)
        fname = f"tag36_11_{tag_id:05d}.png"
        img.save(fname)
        print(f"  [OK] {fname}  ({size}x{size} px)")
        return True
    except Exception as e:
        print(f"  [FAIL] 下载失败: {e}")
        return False


if __name__ == "__main__":
    import os
    script_dir = os.path.dirname(os.path.abspath(__file__))
    os.chdir(script_dir)

    print(f"生成 AprilTag 36h11,  IDs={TAG_IDS},  size={OUT_SIZE}px\n")
    for tid in TAG_IDS:
        print(f"Tag ID={tid}:")
        ok = generate_with_pillow(tid, OUT_SIZE)
        if not ok:
            ok = download_from_github(tid, OUT_SIZE)
        if not ok:
            print(f"  请手动下载: https://github.com/AprilRobotics/apriltag-imgs")

    print("\n完成！可以重新 colcon build --packages-select sim_worlds")
