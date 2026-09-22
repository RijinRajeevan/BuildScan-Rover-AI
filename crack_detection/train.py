# pyrefly: ignore [missing-import]
from ultralytics import YOLO

def main():
    model = YOLO("yolov26n-seg.pt")

    model.train(
        data="data.yaml",
        epochs=50,
        imgsz=640,
        batch=8,
        device=0,          
        patience=50,
        workers=0,
        project="D:/crack_detection",
        name="BuildScan_SegModel",
        exist_ok=True,
        val=True,
        plots=True,
        save=True
    )

if __name__ == "__main__":
    main()