"""
세 개의 plot 이미지를 라벨과 함께 가로로 병합.
사용:
    1) 세 이미지 경로를 IMG_PATHS 에 채움
    2) python merge_3plots.py
출력:
    같은 폴더에 merged_conditions.png
"""

from PIL import Image, ImageDraw, ImageFont
import os

# ── 입력 ─────────────────────────────────────────────────────────
IMG_PATHS = [
    r"D:\HARfinal\plots\notorque.png",
    r"D:\HARfinal\plots\max6.png",
    r"D:\HARfinal\plots\max10.png",
]
LABELS = [
    "No Torque",
    "max Torque = 6",
    "max Torque = 10",
]
OUT_PATH = r"D:\HARfinal\plots\merged_conditions.png"

# ── 설정 ─────────────────────────────────────────────────────────
LABEL_HEIGHT = 80          # 각 이미지 위 라벨 영역 높이 (px)
LABEL_BG = (30, 60, 110)   # 진한 파랑
LABEL_FG = (255, 255, 255) # 흰색
GAP = 10                   # 이미지 사이 간격 (px)
BG = (255, 255, 255)       # 전체 배경 흰색
FONT_SIZE = 48

# ── 폰트 ─────────────────────────────────────────────────────────
def get_font(size):
    for name in [
        "C:/Windows/Fonts/malgun.ttf",   # 맑은고딕
        "C:/Windows/Fonts/arial.ttf",
        "C:/Windows/Fonts/calibri.ttf",
    ]:
        if os.path.exists(name):
            return ImageFont.truetype(name, size)
    return ImageFont.load_default()

font = get_font(FONT_SIZE)

# ── 이미지 로드 + 크기 정규화 ───────────────────────────────────
images = [Image.open(p).convert("RGB") for p in IMG_PATHS]

# 같은 높이로 맞추기 (가장 큰 높이 기준)
target_h = max(im.height for im in images)
resized = []
for im in images:
    if im.height != target_h:
        ratio = target_h / im.height
        new_w = int(im.width * ratio)
        im = im.resize((new_w, target_h), Image.LANCZOS)
    resized.append(im)

# ── 캔버스 ───────────────────────────────────────────────────────
total_w = sum(im.width for im in resized) + GAP * (len(resized) - 1)
total_h = LABEL_HEIGHT + target_h
canvas = Image.new("RGB", (total_w, total_h), BG)
draw = ImageDraw.Draw(canvas)

# ── 배치 ─────────────────────────────────────────────────────────
x_offset = 0
for im, label in zip(resized, LABELS):
    # 라벨 배경 박스
    draw.rectangle(
        [x_offset, 0, x_offset + im.width, LABEL_HEIGHT],
        fill=LABEL_BG
    )
    # 라벨 텍스트 중앙 정렬
    bbox = draw.textbbox((0, 0), label, font=font)
    text_w = bbox[2] - bbox[0]
    text_h = bbox[3] - bbox[1]
    tx = x_offset + (im.width - text_w) // 2
    ty = (LABEL_HEIGHT - text_h) // 2 - bbox[1]
    draw.text((tx, ty), label, fill=LABEL_FG, font=font)

    # 이미지 붙이기
    canvas.paste(im, (x_offset, LABEL_HEIGHT))

    x_offset += im.width + GAP

# ── 저장 ─────────────────────────────────────────────────────────
os.makedirs(os.path.dirname(OUT_PATH), exist_ok=True)
canvas.save(OUT_PATH, dpi=(150, 150))
print(f"Saved: {OUT_PATH}")
print(f"Size: {canvas.size}")
