#!/bin/bash

# Set your own EuRoC_PATH path to run ice-ba. Use "./bin/ice_ba --help" to get the explanation for all of the flags. Flags [imgs_folder] and [iba_param_path] are necessary.
# Add flag '--save_feature' to save feature message and calibration file for back-end only mode

# #euroc
# EuRoC_PATH=~/data/euroc
# DATA_FOLDER=MH_01
# mkdir $EuRoC_PATH/result
# cmd="../bin/ice_ba --imgs_folder $EuRoC_PATH/$DATA_FOLDER --start_idx 0 --end_idx -1 --iba_param_path ../config/config_of_mono.txt  --gba_camera_save_path $EuRoC_PATH/result/$DATA_FOLDER.txt"

#GvDevice
EuRoC_PATH=~/data/GvDevice
DATA_FOLDER=data1
mkdir $EuRoC_PATH/result
cmd="../bin/ice_ba --imgs_folder $EuRoC_PATH/$DATA_FOLDER --start_idx 0 --end_idx -1 --iba_param_path ../config/config_of_mono.txt  --gba_camera_save_path $EuRoC_PATH/result/$DATA_FOLDER.txt"

# #tum_vi
# EuRoC_PATH=~/data/tum_vi
# DATA_FOLDER=dataset-corridor1_512_16
# mkdir $EuRoC_PATH/result
# cmd="../bin/ice_ba --imgs_folder $EuRoC_PATH/$DATA_FOLDER --start_idx 0 --end_idx -1 --iba_param_path ../config/config_of_mono.txt  --gba_camera_save_path $EuRoC_PATH/result/$DATA_FOLDER.txt"

echo $cmd
eval $cmd