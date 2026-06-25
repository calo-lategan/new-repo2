#!/bin/bash
gnome-terminal -- bash -c "rosrun camera_calibration cameracalibrat.py --size=8x6 --square=0.011 image:=/usb_cam/image_raw camera:=/usb_cam; exec bash"
gnome-terminal -- bash -c "ros2 launch  peripherals  usb_cam.launch.py; exec bash" 

