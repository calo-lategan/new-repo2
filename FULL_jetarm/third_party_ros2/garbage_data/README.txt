\data\aJPEGImages  原始图片和标注后的xml文件
\data\images
\data\images\train	 原始图片训练集
\data\images\val   	 原始图片验证集
\data\labels		 dota_txt文件训练集和验证集 txt文件训练集和验证集
\data\val_dota       xml --> dota_xml生成文件
\data\val_original   dota_xml --> dota_txt生成文件
\data\create_training_directories.py   创建训练所需文件夹
\data\dota_to_txt.py				文件转换xml --> dota_xml --> dota_txt
\data\roxml_to_dota.py  			文件转换dota_txt --> txt
\data\split_data.py				将\data\images和\data\val_original文件分90%作为训练集，10%为验证集
\data\create_folders.py					JPEGImages图片进行文件夹分类
\data\data.yaml			训练需要调用的yaml文件
\data\images_360.py		将原始图片旋转360度，每隔2度生成1张
\data\trans_xml.py		将原始图片旋转360度，每隔2度生成1份
\data\yolov8_train.py	开始训练yolov8-obb模型
\data\yolov8n-obb.pt	训练需要用到的权重文件
