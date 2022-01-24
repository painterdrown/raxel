#!/bin/bash

CALIB_PATH=/home/zhengzhao/gcalib/camera_calibration
DATA_PATH=$1
HALF_WINDOW_SIZE=15

${CALIB_PATH}/build/applications/camera_calibration/camera_calibration \
    --pattern_files ${CALIB_PATH}/../patterns/pattern_resolution_17x24_segments_16_apriltag_0.yaml \
    --image_directories ${DATA_PATH} \
    --dataset_output_path ${DATA_PATH}/features_${HALF_WINDOW_SIZE}px.bin \
    --refinement_window_half_extent ${HALF_WINDOW_SIZE} \
    --show_visualizations  # optional for showing visualizations
#    --no_cuda_feature_detection  # use this to disable using CUDA for feature detection
