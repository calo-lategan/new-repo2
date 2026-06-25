import os
import xml.etree.ElementTree as ET

# 定义类别映射，根据需要修改
CLASS_MAPPING = {
    "Paizi": 0,  # "Paizi" 对应的 class_id 是 0
    # 添加其他类别映射，例如 "AnotherClass": 1
}

# 归一化坐标
def normalize_coordinates(cx, cy, w, h, img_width, img_height):
    norm_cx = cx / img_width
    norm_cy = cy / img_height
    norm_w = w / img_width
    norm_h = h / img_height
    return norm_cx, norm_cy, norm_w, norm_h

# 转换单个 XML 文件为 YOLO 格式
def convert_xml_to_txt(xml_file, output_dir):
    tree = ET.parse(xml_file)
    root = tree.getroot()

    # 获取图片尺寸
    img_width = int(root.find('size/width').text)
    img_height = int(root.find('size/height').text)

    # 获取文件名
    filename = os.path.splitext(root.find('filename').text)[0]
    txt_file_path = os.path.join(output_dir, f"{filename}.txt")

    with open(txt_file_path, 'w') as txt_file:
        for obj in root.findall('object'):
            class_name = obj.find('name').text
            if class_name not in CLASS_MAPPING:
                print(f"Warning: class {class_name} not in CLASS_MAPPING, skipping.")
                continue

            class_id = CLASS_MAPPING[class_name]
            robndbox = obj.find('robndbox')

            cx = float(robndbox.find('cx').text)
            cy = float(robndbox.find('cy').text)
            w = float(robndbox.find('w').text)
            h = float(robndbox.find('h').text)

            # 归一化
            norm_cx, norm_cy, norm_w, norm_h = normalize_coordinates(cx, cy, w, h, img_width, img_height)

            # 写入文件，YOLO 格式：class_id cx cy width height
            txt_file.write(f"{class_id} {norm_cx:.6f} {norm_cy:.6f} {norm_w:.6f} {norm_h:.6f}\n")

    print(f"Converted {xml_file} to {txt_file_path}")

# 批量转换 XML 文件
def convert_all_xml_in_dir(xml_dir, output_dir):
    os.makedirs(output_dir, exist_ok=True)
    for file_name in os.listdir(xml_dir):
        if file_name.endswith('.xml'):
            xml_file_path = os.path.join(xml_dir, file_name)
            convert_xml_to_txt(xml_file_path, output_dir)

if __name__ == "__main__":
    # 输入 XML 文件目录
    input_xml_dir = "/home/ubuntu/third_party_ros2/data/Annotations"
    # 输出 TXT 文件目录
    output_txt_dir = "/home/ubuntu/third_party_ros2/data/JPEGImages"

    convert_all_xml_in_dir(input_xml_dir, output_txt_dir)

