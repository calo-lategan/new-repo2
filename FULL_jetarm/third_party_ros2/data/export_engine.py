from ultralytics import YOLO

# Export the trained model to a TensorRT .engine for the live Jetson demo.
# IMPORTANT: run this ON THE JETSON - a .engine is specific to that GPU + TensorRT version.
# imgsz MUST match training (640).

model = YOLO('/home/ubuntu/third_party_ros2/data/scaff_cubes_obb_v2.pt')

model.export(
    format='engine',
    imgsz=640,
    half=True,     # FP16 - fast and accurate enough on Orin Nano. Use int8=True only with a calibration set.
    device=0,
)
# Produces best.engine next to best.pt
