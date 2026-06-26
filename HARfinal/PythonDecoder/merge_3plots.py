"""
세 condition 의 plot 이미지를 라벨과 함께 가로로 병합.
출력: merged_conditions.png
"""

from PIL import Image, ImageDraw, ImageFont
import os

# 입력 ───────────────────────────────────────────────────────────
BASE = r"D:\HARfinal2\HARfinal\PythonDecoder"
IMG_PATHS = [
    os.path.join(BASE, "notorque.jpg"),
    os.path.join(BASE, "16B.jpg"),
    os.path.join(BASE, "16E.jpg"),
]
LABELS = [
    "No Torque",
    "max Torque = 6",
    "max Torque = 10",
]
OUT_PATH = os.path.join(BASE, "merged_conditions.png")

# 설정 ───────────────────────────────────────────────────────────
LABEL_HEIGHT = 80
LABEL_BG = (30, 60, 110)
LABEL_FG = (255, 255, 255)
GAP = 12
BG = (255, 255, 255)
FONT_SIZE = 48

def get_font(size):
    for name in [
        "C:/Windows/Fonts/malgun.ttf",
        "C:/Windows/Fonts/arial.ttf",
        "C:/Windows/Fonts/calibri.ttf",
    ]:
        if os.path.exists(name):
            return ImageFont.truetype(name, size)
    return ImageFont.load_default()

font = get_font(FONT_SIZE)

# 이미지 로드 + 같은 높이로 정규화 ───────────────────────────────
images = [Image.open(p).convert("RGB") for p in IMG_PATHS]
target_h = max(im.height for im in images)
resized = []
for im in images:
    if im.height != target_h:
        ratio = target_h / im.height
        new_w = int(im.width * ratio)
        im = im.resize((new_w, target_h), Image.LANCZOS)
    resized.append(im)

# 캔버스 ─────────────────────────────────────────────────────────
total_w = sum(im.width for im in resized) + GAP * (len(resized) - 1)
total_h = LABEL_HEIGHT + target_h
canvas = Image.new("RGB", (total_w, total_h), BG)
draw = ImageDraw.Draw(canvas)

x_offset = 0
for im, label in zip(resized, LABELS):
    draw.rectangle(
        [x_offset, 0, x_offset + im.width, LABEL_HEIGHT],
        fill=LABEL_BG
    )
    bbox = draw.textbbox((0, 0), label, font=font)
    text_w = bbox[2] - bbox[0]
    text_h = bbox[3] - bbox[1]
    tx = x_offset + (im.width - text_w) // 2
    ty = (LABEL_HEIGHT - text_h) // 2 - bbox[1]
    draw.text((tx, ty), label, fill=LABEL_FG, font=font)
    canvas.paste(im, (x_offset, LABEL_HEIGHT))
    x_offset += im.width + GAP

canvas.save(OUT_PATH, dpi=(150, 150))
print(f"Saved: {OUT_PATH}")
print(f"Size: {canvas.size}")
