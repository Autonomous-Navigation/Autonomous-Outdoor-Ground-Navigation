#! /bin/bash

echo "admin" | sudo chmod 777 /dev/ttyUSB0
roslaunch rover agni_final.launch model_name:=fcn-resnet18-cityscapes-512x256 input_width:=512 input_height:=256 output:=display://0
