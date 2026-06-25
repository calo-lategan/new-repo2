import glob

# Search in JPEGImages and the split train/val folders just in case
search_paths = [
    '/home/ubuntu/third_party_ros2/data/JPEGImages/*.txt',
    '/home/ubuntu/third_party_ros2/data/labels/train/*.txt',
    '/home/ubuntu/third_party_ros2/data/labels/val/*.txt'
]

for path in search_paths:
    for txt_file in glob.glob(path):
        if "classes" in txt_file: continue
        with open(txt_file, 'r') as f:
            lines = f.readlines()
        
        with open(txt_file, 'w') as f:
            for line in lines:
                parts = line.strip().split()
                if len(parts) == 5:
                    c, cx, cy, w, h = map(float, parts)
                    # Calculate the 4 corners
                    x1 = cx - w/2
                    y1 = cy - h/2
                    x2 = cx + w/2
                    y2 = cy - h/2
                    x3 = cx + w/2
                    y3 = cy + h/2
                    x4 = cx - w/2
                    y4 = cy + h/2
                    f.write(f"{int(c)} {x1:.6f} {y1:.6f} {x2:.6f} {y2:.6f} {x3:.6f} {y3:.6f} {x4:.6f} {y4:.6f}\n")
                else:
                    f.write(line)
print("All boxes successfully converted to 8-coordinate OBB format!")
