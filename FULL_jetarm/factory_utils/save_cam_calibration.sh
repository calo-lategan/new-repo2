
mkdir -p calibration_data 

tar -xzvf /tmp/calibrationdata.tar.gz -C /home/ubuntu/factory_utils/calibration_data

cd calibration_data

sed -i 's/camera_name: narrow_stereo/camera_name: usb_cam/' ost.yaml

mv ost.yaml camera_info.yaml

cp camera_info.yaml /home/ubuntu/ros2_ws/src/peripherals/config

rm -rf /home/ubuntu/factory_utils/calibration_data

