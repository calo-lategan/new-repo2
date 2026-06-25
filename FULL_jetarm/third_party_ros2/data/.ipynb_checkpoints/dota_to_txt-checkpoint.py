import sys
 
sys.path.append('/home/featurize/work/ultralytics/')
 
from ultralytics.data.converter import convert_dota_to_yolo_obb
convert_dota_to_yolo_obb('/home/featurize/work/adata')
