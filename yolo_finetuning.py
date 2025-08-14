#REMINDER. This code is only apropriate for Google Colab.

#Chenck if T4 videocard environment is available
!nvidia-smi

#Install YOLO library
!pip install ultralytics

#Install Roboflow libarry
!pip install roboflow

#Download our own dataset from Roboflow to Google Colab. The data split proccess happens there
from roboflow import Roboflow
rf = Roboflow(api_key="cKjRecfQA5dG67anfGxr")
project = rf.workspace("objectdetectiondata").project("waste_dataset-b8s4n")
version = project.version(2)
dataset = version.download("yolov11")


#Start YOLO fine-tuning. .pt means pre-trained model
#Don't forget to setup data.yaml train/validation/test directions
!yolo detect train data=/content/waste_dataset-2/data.yaml model=yolo11s.pt epochs=60 imgsz=640

#Get all fine-tuning resulst and graphs (validation images)
!yolo detect predict model=runs/detect/train/weights/best.pt source=waste_dataset-2/valid/images save=True

#Get fine-tuning results and graph (test images)
!yolo val \
  model=/content/runs/detect/train/weights/best.pt \
  data=/content/waste_dataset-2/data.yaml \
  split=test imgsz=640 \
  project=/content/runs/detect name=val_test exist_ok=True
