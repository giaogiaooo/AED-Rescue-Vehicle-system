import warnings
warnings.filterwarnings('ignore')

from ultralytics import YOLO

if __name__ == '__main__':

    model = YOLO("yolov8n.pt")

    model.train(
        data=r"E:/super ubantu/fall/train/falldata.yaml",

        epochs=150,

        imgsz=640,

        batch=32,

        workers=0,

        device=0,

        amp=True,

        cache=False,

        optimizer='AdamW',

        lr0=0.001,

        patience=20,

        augment=True,

        mosaic=0.5,

        mixup=0.1,

        close_mosaic=10
    )