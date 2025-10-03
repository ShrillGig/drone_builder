from ultralytics import YOLO
import cv2
from pathlib import Path
import numpy as np
import matplotlib.pyplot as plt


def iou(frame, ground):
    # бинаризируем изображение
    frame = frame / 255
    ground = ground / 255

    # данные формулы применимы исключительно для сегментации
    tp = (frame * ground).sum()
    fp = ((frame + ground) - frame).sum()
    fn = ((frame + ground) - ground).sum()

    iou = tp / (tp + fp + fn)

    return iou


def core(images, masks, output, index):

    iou_number = 0
    iou_stack = []
    conf_stack = []
    min_conf = 0.557
    model_path = 'best.pt'
    model = YOLO(model_path, task="detect")

    output_image = output / f"video_image{index}.mp4"
    output_mask = output / f"video_mask{index}.mp4"
    output_thresh = output / f"video_thresh{index}.mp4"
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    out_image = cv2.VideoWriter(str(output_image), fourcc, 30, (640, 640))
    out_mask = cv2.VideoWriter(str(output_mask), fourcc, 30, (640, 640))
    out_thresh = cv2.VideoWriter(str(output_thresh), fourcc, 30, (640, 640))

    for image, mask in zip(images.iterdir(), masks.iterdir()):
         if image.suffix.lower() == ".jpg":
            frame_image = cv2.imread(str(image))
            frame_mask = cv2.imread(str(mask))

            results = model.predict(frame_image, verbose=False)
            detections = results[0].boxes

            if detections is not None and len(detections) > 0:

                x1, y1, x2, y2 = detections.xyxy[0].int().cpu().tolist()
                conf = detections[0].conf.item()
                new_image = results[0].plot()

            else:
                new_image = frame_image

            gray_image = cv2.cvtColor(new_image, cv2.COLOR_BGR2GRAY)
            frameROI = gray_image[y1:y2,x1:x2]  # динамическая рамка благодаря object detection для фокуса threshold
            blur = cv2.GaussianBlur(frameROI, (5, 5), 0)
            adapt_thresh = cv2.adaptiveThreshold(blur, 255, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY_INV, 21,10)
            black_image = np.zeros([640, 640], np.uint8)  # создаем пустое черное изображение 640х640
            black_image[y1:y2,x1:x2] = adapt_thresh  # накладываем на черное изображение рамку с threshold, чтобы фон не мешал при IoU
            color_image = cv2.cvtColor(black_image, cv2.COLOR_GRAY2BGR)

            iou_number = iou(color_image, frame_mask)
            cv2.putText(color_image, f"IoU:{iou_number:.2f}", (30, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5,(255, 255, 255), 2)
            iou_stack.append(iou_number)
            conf_stack.append(conf)

            out_thresh.write(color_image)
            out_image.write(new_image)
            out_mask.write(frame_mask)

    out_image.release()
    out_mask.release()
    out_thresh.release()

    return iou_stack, conf_stack

def main():

    index = 0
    iou_mean = []
    conf_mean = []

    folder_image = Path("images")
    folder_mask = Path("masks")
    output = Path("output_image")
    output.mkdir(exist_ok=True)

    for images, masks in zip(folder_image.iterdir(), folder_mask.iterdir()):

        iou_stack, conf_stack = core(images, masks, output, index)
        iou_mean.append(sum(iou_stack) / len(iou_stack))
        conf_mean.append(sum(conf_stack) / len(conf_stack))
        index += 1

    print(f"IOU{index}: {iou_mean} ---- Conf{index}: {conf_mean}")

if __name__ == "__main__":
    main()






