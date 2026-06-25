from ultralytics import YOLO

# Run 2: same dataset, extra augmentation aimed at real-world / live-demo robustness.
# Writes to a NEW run folder (scaff_cubes_obb_v2) so your run-1 weights are untouched.

# Nano (what you have). To try the larger model for the harder classes, swap the line below
# to:  model = YOLO('yolo26s-obb.pt')   -- only if the Orin Nano still holds real-time FPS.
model = YOLO('yolo26n-obb.pt')

results = model.train(
    data='/home/ubuntu/third_party_ros2/data/data.yaml',
    epochs=100,
    imgsz=640,
    batch=4,            # Orin Nano 8GB. Try 8 if it fits; drop to 2 on CUDA OOM.
    amp=False,
    cache=False,
    workers=4,
    device=0,
    patience=30,

    # --- the additions over run 1 (these are NOT Ultralytics defaults) ---
    degrees=10.0,       # random rotation - teaches orientation invariance (great for OBB)
    translate=0.2,      # up from default 0.1 - more half-in-frame / off-center cases
    mixup=0.1,          # blends image pairs - helps crowded scenes & background FPs
    copy_paste=0.1,     # note: mainly affects segmentation; often a no-op for OBB, harmless

    # defaults already on in run 1 (listed for clarity - no need to change):
    # fliplr=0.5, flipud=0.0, hsv_h=0.015, hsv_s=0.7, hsv_v=0.4, scale=0.5, mosaic=1.0

    project='/home/ubuntu/third_party_ros2/data/runs',
    name='scaff_cubes_obb_v2',
)

print("Best weights:", model.trainer.best)
