import os
import xml.etree.ElementTree as ET

# Maps your custom label exactly
CLASS_MAPPING = {
    "scaff": 0,
}

def normalize_coordinates(cx, cy, w, h, img_width, img_height):
    norm_cx = cx / img_width
    norm_cy = cy / img_height
    norm_w = w / img_width
    norm_h = h / img_height
    return norm_cx, norm_cy, norm_w, norm_h

def convert_xml_to_txt(xml_file, output_dir):
    tree = ET.parse(xml_file)
    root = tree.getroot()

    img_width = int(root.find('size/width').text)
    img_height = int(root.find('size/height').text)
    filename = os.path.splitext(root.find('filename').text)[0]
    txt_file_path = os.path.join(output_dir, f"{filename}.txt")

    with open(txt_file_path, 'w') as txt_file:
        for obj in root.findall('object'):
            class_name = obj.find('name').text
            if class_name not in CLASS_MAPPING:
                continue

            class_id = CLASS_MAPPING[class_name]
            
            bndbox = obj.find('bndbox')
            robndbox = obj.find('robndbox')

            # This automatically converts your standard boxes to YOLO format
            if bndbox is not None:
                xmin = float(bndbox.find('xmin').text)
                ymin = float(bndbox.find('ymin').text)
                xmax = float(bndbox.find('xmax').text)
                ymax = float(bndbox.find('ymax').text)
                w = xmax - xmin
                h = ymax - ymin
                cx = xmin + (w / 2.0)
                cy = ymin + (h / 2.0)
            elif robndbox is not None:
                cx = float(robndbox.find('cx').text)
                cy = float(robndbox.find('cy').text)
                w = float(robndbox.find('w').text)
                h = float(robndbox.find('h').text)
            else:
                continue

            norm_cx, norm_cy, norm_w, norm_h = normalize_coordinates(cx, cy, w, h, img_width, img_height)
            txt_file.write(f"{class_id} {norm_cx:.6f} {norm_cy:.6f} {norm_w:.6f} {norm_h:.6f}\n")

    print(f"Converted {xml_file} to {txt_file_path}")

def convert_all_xml_in_dir(xml_dir, output_dir):
    os.makedirs(output_dir, exist_ok=True)
    for file_name in os.listdir(xml_dir):
        if file_name.endswith('.xml'):
            xml_file_path = os.path.join(xml_dir, file_name)
            convert_xml_to_txt(xml_file_path, output_dir)

if __name__ == "__main__":
    input_xml_dir = "/home/ubuntu/third_party_ros2/data/Annotations"
    output_txt_dir = "/home/ubuntu/third_party_ros2/data/JPEGImages"
    convert_all_xml_in_dir(input_xml_dir, output_txt_dir)
