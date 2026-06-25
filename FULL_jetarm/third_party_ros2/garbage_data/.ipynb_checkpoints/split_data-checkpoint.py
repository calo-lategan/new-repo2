import os
import shutil
import random

def clear_directory(directory_path):
    """清空指定目录中的所有文件和子目录"""
    if os.path.exists(directory_path):
        shutil.rmtree(directory_path)
    os.makedirs(directory_path, exist_ok=True)

def split_data(base_path,  val_ratio=0.1):
    # 设置路径
    images_base_path = os.path.join(base_path, 'JPEGImages')
    labels_base_path = os.path.join(base_path, 'val_original')
    val_images_path = os.path.join(base_path, 'images/val')
    train_images_path = os.path.join(base_path, 'images/train')
    val_labels_path = os.path.join(base_path, 'labels/val_original')
    train_labels_path = os.path.join(base_path, 'labels/train_original')

    # 清空目标目录
    clear_directory(val_images_path)
    clear_directory(train_images_path)
    clear_directory(val_labels_path)
    clear_directory(train_labels_path)


    # 获取文件夹路径
    image_folder = os.path.join('/home/featurize/work/data/JPEGImages')
    label_folder = os.path.join('/home/featurize/work/data/val_original/')

    # 获取图像和标签文件列表
    image_files = [f for f in os.listdir(image_folder) if f.endswith('.jpg')]
    label_files = [f for f in os.listdir(label_folder) if f.endswith('.txt')]

    # 确定数量
    total_images = len(image_files)
    val_count = max(1, int(total_images * val_ratio))  # 至少抽取1张
    train_count = total_images - val_count

    # 随机选择验证集
    val_indices = set(random.sample(range(total_images), val_count))

    # 将文件移动到相应的文件夹
    for idx, image_file in enumerate(image_files):
        base_name = os.path.splitext(image_file)[0]
        image_src = os.path.join(image_folder, image_file)
        label_src = os.path.join(label_folder, f'{base_name}.txt')

        # 检查文件是否存在，避免报错
        if not os.path.exists(image_src):
            print(f"警告: 图像文件不存在: {image_src}")
            continue

        if not os.path.exists(label_src):
            print(f"警告: 标签文件不存在: {label_src}")
            continue

        if idx in val_indices:
            # 复制到验证集
            shutil.copy(image_src, val_images_path)
            shutil.copy(label_src, val_labels_path)
        else:
            # 复制到训练集
            shutil.copy(image_src, train_images_path)
            shutil.copy(label_src, train_labels_path)

    print(f': 训练集 {train_count} 张, 验证集 {val_count} 张')

if __name__ == "__main__":
    base_path = "/home/featurize/work/data"    #要修改为自己的文件路径
 
    split_data(base_path,)
