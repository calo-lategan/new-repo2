#!/bin/zsh
source /home/ubuntu/.zshrc
gnome-terminal -- bash -c "python3 /home/ubuntu/factory_utils/calibration/calibration.py; exec bash"
gnome-terminal -- bash -c "python3 /home/ubuntu/factory_utils/calibration/main.py; exec bash" 


