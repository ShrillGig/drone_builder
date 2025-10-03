from ultralytics import YOLO
import cv2
from pathlib import Path
import numpy as np
import matplotlib.pyplot as plt


output_path = Path("C:/Users/F1/Desktop/video/video_coco3_obdet_fine")
model_path = 'best.pt'

output_path.mkdir(exist_ok=True)


model = YOLO(model_path, task="detect")

i = 0
min_conf = 0.557
frame_count = 0
max_list = []
other_list = []
class_list = []
none_list = []
max_frames = 210
video_folder = Path("video")

for file in video_folder.iterdir():

    conf_list = []
    confidence = {}
    cam = cv2.VideoCapture(str(file))
    #frame_count = int(cam.get(cv2.CAP_PROP_FRAME_COUNT))
    image_width = int(cam.get(cv2.CAP_PROP_FRAME_WIDTH))
    image_height = int(cam.get(cv2.CAP_PROP_FRAME_HEIGHT))

    output_folder = output_path / f"video_edit{i}.mp4"
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    fps = cam.get(cv2.CAP_PROP_FPS)
    out = cv2.VideoWriter(str(output_folder), fourcc, fps, (image_width, image_height))


    while True:

        ret, frame = cam.read()

        if not ret:
            print(f"\n{file} is done")

            for key, value in confidence.items():
                mean = np.mean(value)
                count = len(value)
                confidence[key] = {"mean":mean, "count": count}

            print(f"\nDictionary {file}: {confidence}")


            other_sum = 0
            maxs = 0
            for key, _ in confidence.items():
                if confidence[key]['count'] > maxs:
                    maxs = confidence[key]['count']
                    buf = key

            for key, _ in confidence.items():
                if confidence[key]['count'] < maxs:
                    other_sum += confidence[key]['count']

            none_number = max_frames - (maxs + other_sum)

            max_list.append(maxs)
            other_list.append(other_sum)
            none_list.append(none_number)
            class_list.append(buf)

            i += 1
            break

        results = model.predict(frame, verbose=False)
        detections = results[0].boxes

        if detections is not None and len(detections) > 0:

            annotated = results[0].plot()
            conf = detections[0].conf.item()
            out.write(annotated)

            name = model.names[int(detections[0].cls)]
            confidence.setdefault(name, []).append(conf)
        else:
            out.write(frame)



    cam.release()
    out.release()
    cv2.destroyAllWindows()

print(f"Max list:{max_list}")
print(f"Classes: {class_list}")
print(f"Other frames list {other_list}")

plt.axhline(y=210, color="red", linestyle="--", label="Maximum frames")
plt.bar(range(0, i), max_list, label="Most frequent class")
# plt.bar(range(0,i), other_list, bottom = max_list, color = "orange", label = "Other detected classes")
plt.bar(range(0,i), none_list, bottom = np.array(max_list) + np.array(other_list), color="gray", label = "No detections")
# подписи в центре сегментов
for i, v in enumerate(max_list):
    plt.text(i, v/2, str(v), ha='center', color="white")   # середина синего блока

# for i, v in enumerate(other_list):
#     plt.text(i, max_list[i] + v/2, str(v), ha='center', color="black")   # середина оранжевого блока

for i, v in enumerate(none_list):
    plt.text(i, max_list[i] + other_list[i] + v/2, str(v), ha='center', color="black")

plt.ylim(0, 300)
plt.legend()
plt.title('YOLO frame-level detections (fine-tuned on custom data)')
plt.xlabel('Videos')
plt.ylabel('Frame count')
plt.show()
