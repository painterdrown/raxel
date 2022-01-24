#!/bin/bash

# 算法参数
HALF_WINDOW_SIZE=15
MODEL=central_generic  # Or noncentral_generic, central_opencv
CELL_SIZE=20  # Choose a suitable value for the camera's resolution
NUM_PYRAMID_LEVELS=4
# NUM_PYRAMID_LEVELS=1

CALIB_PATH=/home/zhengzhao/gcalib/camera_calibration
DATA_PATH=$1

${CALIB_PATH}/build/applications/camera_calibration/camera_calibration \
    --dataset_files ${DATA_PATH}/features_${HALF_WINDOW_SIZE}px.bin \
    --output_directory ${DATA_PATH}/result_${HALF_WINDOW_SIZE}px_${MODEL}_${CELL_SIZE} \
    --cell_length_in_pixels ${CELL_SIZE} \
    --model ${MODEL} \
    --num_pyramid_levels ${NUM_PYRAMID_LEVELS}

# For gdb debugging
# gdb --ex run --args ${CALIB_PATH}/build/applications/camera_calibration/camera_calibration \
#     --dataset_files ${DATA_PATH}/features_${HALF_WINDOW_SIZE}px.bin \
#     --output_directory ${DATA_PATH}/result_${HALF_WINDOW_SIZE}px_${MODEL}_${CELL_SIZE} \
#     --cell_length_in_pixels ${CELL_SIZE} \
#     --model ${MODEL} \
#     --num_pyramid_levels ${NUM_PYRAMID_LEVELS}
