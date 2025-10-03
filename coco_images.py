import json
import cv2
import numpy as np
from pathlib import Path

# путь к COCO json
with open("_annotations.coco.json", "r") as f:
    coco = json.load(f)

images = {img["id"]: img for img in coco["images"]}

output_dir = Path("masks")
output_dir.mkdir(exist_ok=True)

for ann in coco["annotations"]:
    img_info = images[ann["image_id"]]
    width, height = img_info["width"], img_info["height"]
    filename = img_info["file_name"]

    # создаем пустую маску
    mask = np.zeros((height, width), dtype=np.uint8)

    # segmentation может быть списком полигонов
    for seg in ann["segmentation"]:
        pts = np.array(seg).reshape(-1, 2).astype(np.int32)
        cv2.fillPoly(mask, [pts], 1)

    # сохраняем маску
    out_path = output_dir / filename.replace(".jpg", "_mask.png")
    cv2.imwrite(str(out_path), mask * 255)  # 0/255 для бинарной маски
