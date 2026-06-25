import os
import shutil
import json
import xml.etree.ElementTree as ET
from xml.dom import minidom

def clear_directory(directory_path):
    """Deletes and recreates a directory to ensure it is empty."""
    if os.path.exists(directory_path):
        shutil.rmtree(directory_path)
    os.makedirs(directory_path, exist_ok=True)

def create_pascal_voc_xml(xml_path, folder, filename, width, height, obb_annotations):
    """Generates legacy XML files for the Annotations folder."""
    root = ET.Element("annotation")
    ET.SubElement(root, "folder").text = folder
    ET.SubElement(root, "filename").text = filename
    
    size = ET.SubElement(root, "size")
    ET.SubElement(size, "width").text = str(width)
    ET.SubElement(size, "height").text = str(height)
    ET.SubElement(size, "depth").text = "3"
    
    for obb in obb_annotations:
        class_id = obb[0]
        coords = obb[1:] # The 8 normalized coordinates
        
        # Denormalize coordinates to absolute pixels
        xs = [coords[0]*width, coords[2]*width, coords[4]*width, coords[6]*width]
        ys = [coords[1]*height, coords[3]*height, coords[5]*height, coords[7]*height]
        
        # Create standard bounding box to satisfy XML format
        xmin, xmax = min(xs), max(xs)
        ymin, ymax = min(ys), max(ys)
        
        obj = ET.SubElement(root, "object")
        ET.SubElement(obj, "name").text = str(class_id)
        ET.SubElement(obj, "pose").text = "Unspecified"
        ET.SubElement(obj, "truncated").text = "0"
        ET.SubElement(obj, "difficult").text = "0"
        
        bndbox = ET.SubElement(obj, "bndbox")
        ET.SubElement(bndbox, "xmin").text = str(int(xmin))
        ET.SubElement(bndbox, "ymin").text = str(int(ymin))
        ET.SubElement(bndbox, "xmax").text = str(int(xmax))
        ET.SubElement(bndbox, "ymax").text = str(int(ymax))
        
    xml_str = minidom.parseString(ET.tostring(root)).toprettyxml(indent="    ")
    with open(xml_path, "w") as f:
        f.write(xml_str)

def process_and_split_data(base_path):
    ndjson_file = os.path.join(base_path, 'all-annotated.ndjson')
    source_folder = os.path.join(base_path, 'JPEGImages')
    xml_folder = os.path.join(base_path, 'Annotations')
    
    # 1. Create Annotations folder
    os.makedirs(xml_folder, exist_ok=True)

    # 2. Set the exact destinations YOLO expects and clear them
    splits = ['train', 'val', 'test']
    paths = {}
    for s in splits:
        paths[f'{s}_img'] = os.path.join(base_path, f'images/{s}')
        paths[f'{s}_lbl'] = os.path.join(base_path, f'labels/{s}')
        clear_directory(paths[f'{s}_img'])
        clear_directory(paths[f'{s}_lbl'])

    if not os.path.exists(ndjson_file):
        print(f"Error: {ndjson_file} not found in {base_path}!")
        return

    counts = {'train': 0, 'val': 0, 'test': 0}
    print("Parsing JSON, writing TXT/XMLs, and routing to split folders...")
    
    with open(ndjson_file, 'r') as f:
        for line in f:
            data = json.loads(line.strip())
            
            if data.get("type") == "image":
                filename = data["file"]
                split = data.get("split", "train") # Pulls explicitly from JSON
                width = data.get("width", 640)
                height = data.get("height", 480)
                base_name = os.path.splitext(filename)[0]
                
                image_src = os.path.join(source_folder, filename)
                label_src = os.path.join(source_folder, f'{base_name}.txt')
                xml_dst = os.path.join(xml_folder, f'{base_name}.xml')
                
                # Check if image actually exists in JPEGImages before processing
                if not os.path.exists(image_src):
                    print(f"Warning: {filename} not found in JPEGImages. Skipping.")
                    continue
                
                annotations = data.get("annotations", {})
                obb_list = annotations.get("obb", [])
                
                # Step A: Write the TXT file directly into JPEGImages
                with open(label_src, 'w') as label_file:
                    for obb in obb_list:
                        class_id = obb[0]
                        coords = " ".join([f"{coord:.5f}" for coord in obb[1:]])
                        label_file.write(f"{class_id} {coords}\n")
                        
                # Step B: Write the XML file directly into Annotations
                create_pascal_voc_xml(xml_dst, 'JPEGImages', filename, width, height, obb_list)
                
                # Step C: Copy files to their final Train/Val/Test destination (FIXED)
                if split in ['train', 'val', 'test']:
                    shutil.copy(image_src, paths[f'{split}_img'])
                    shutil.copy(label_src, paths[f'{split}_lbl'])
                    counts[split] += 1

    print(f"\nSUCCESS! Dataset Processed and Split.")
    print(f"Training: {counts['train']} | Validation: {counts['val']} | Test: {counts['test']}")

if __name__ == "__main__":
    base_path = "/home/ubuntu/third_party_ros2/data"
    process_and_split_data(base_path)
