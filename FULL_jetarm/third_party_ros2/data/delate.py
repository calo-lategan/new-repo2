import os
import xml.etree.ElementTree as ET

def update_xml_file(xml_file, prefix, directory):
    """
    更新XML文件中的 <filename> 和 <path> 元素。
    
    :param xml_file: XML文件的路径
    :param prefix: 新的文件名前缀
    :param directory: 文件所在目录的路径
    """
    tree = ET.parse(xml_file)
    root = tree.getroot()
    
    # 查找 <filename> 和 <path> 元素
    filename_element = root.find('filename')
    path_element = root.find('path')
    
    if filename_element is not None and path_element is not None:
        # 提取当前编号
        file_number = filename_element.text.split('_')[-1].split('.')[0]
        
        # 构造新的文件名和路径
        new_filename = f'{prefix}_{file_number}.jpg'
        new_path = os.path.join(directory, new_filename)
        
        # 更新XML中的值
        filename_element.text = new_filename
        path_element.text = new_path
        
        # 保存更新后的XML文件
        tree.write(xml_file, encoding='utf-8', xml_declaration=True)
        print(f'Updated: {xml_file}')

# 指定目录路径和前缀
xml_directory = '/home/ubuntu/third_party_ros2/data/JPEGImages/umbrella_xml'
new_prefix = 'umbrella'
new_directory = '/home/ubuntu/third_party_ros2/data/JPEGImages/umbrella_xml'

# 遍历目录中的所有XML文件并更新
for filename in os.listdir(xml_directory):
    if filename.endswith('.xml'):
        xml_file_path = os.path.join(xml_directory, filename)
        update_xml_file(xml_file_path, new_prefix, new_directory)
