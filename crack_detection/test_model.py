from ultralytics import YOLO

# Load TorchScript model
model = YOLO(r"D:\crack_detection\BuildScan_SegModel\weights\best.pt")

# Predict on test folder
results = model.predict(
    source=r"D:\crack_detection\datasets\crack-seg\images\test",
    conf=0.25,
    save=True,
    imgsz=640,
)

print("Prediction Completed")