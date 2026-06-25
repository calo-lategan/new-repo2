from ultralytics import YOLO
# Load a model
model = YOLO('yolov8s-obb.pt')  
# Train the model
# results = model.train(data='/home/ubuntu/third_party_ros2/data/data.yaml', epochs=50, batch=10, imgsz=640,amp=False)

results = model.train(data='/home/featurize/work/data/data.yaml',epochs=600, batch=64, imgsz=640)




