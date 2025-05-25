from ultralytics import YOLO
import multiprocessing
import cv2
import torch

torch.backends.cudnn.benchmark = True
torch.backends.cudnn.enabled = False

if __name__ == '__main__':
    multiprocessing.freeze_support() # Ensure this is the first line in the if block

    # Load a model
    model = YOLO("best_12v_61.pt").to("cuda")  # Load a model from file

    # Use the model
    model.train(data="config.yaml",
                 epochs=100,
                 imgsz=512,
                 batch=8,
                 device="cuda",
                 optimizer="AdamW",
                 patience=10,
                 amp= False,
                 #workers=8,
                 lr0=0.001,
                 weight_decay=0.0005,
                 augment=False)  # Train the model

    metrics = model.val()  # evaluate model performance on the validation set