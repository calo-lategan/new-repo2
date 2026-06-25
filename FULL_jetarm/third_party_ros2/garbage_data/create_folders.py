import os

# 定义路径
rol_image_dir = '/home/ubuntu/third_party_ros2/data/JPEGImages/rol_image'
jpegimages_dir = '/home/ubuntu/third_party_ros2/data/JPEGImages'

# 获取所有 .jpg 和 .xml 文件的列表
jpg_files = [f for f in os.listdir(rol_image_dir) if f.endswith('.jpg')]
xml_files = [f for f in os.listdir(rol_image_dir) if f.endswith('.xml')]

# 检查并创建所需的目录
for jpg_file in jpg_files:
    # 获取文件名前缀
    base_name = os.path.splitext(jpg_file)[0]
    
    # 检查并创建前缀_train 文件夹
    train_folder = os.path.join(jpegimages_dir, f'{base_name}_train')
    if not os.path.exists(train_folder):
        os.makedirs(train_folder)
        print(f"创建了文件夹: {train_folder}")

for xml_file in xml_files:
    # 获取文件名前缀
    base_name = os.path.splitext(xml_file)[0]
    
    # 检查并创建前缀_xml 文件夹
    xml_folder = os.path.join(jpegimages_dir, f'{base_name}_xml')
    if not os.path.exists(xml_folder):
        os.makedirs(xml_folder)
        print(f"创建了文件夹: {xml_folder}")
