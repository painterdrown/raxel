#!/bin/bash

# 1. 需传入标定数据路径 DATA_PATH 作为 $1
# 2. DATA_PATH 下新建目录 images0 和 images1，分别存放测距图片的左右图
# 3. 测距图片应为 PNG 格式，以 .png 结尾，文件名（不带后缀）作为 $2

# 算法参数
HALF_WINDOW_SIZE=15
MODEL=central_generic  # noncentral_generic
CELL_SIZE=50  # Choose a suitable value for the camera's resolution

CALIB_PATH=/home/zhengzhao/gcalib/camera_calibration
DATA_PATH=$1
CALIB_RESULT=${DATA_PATH}/result_${HALF_WINDOW_SIZE}px_${MODEL}_${CELL_SIZE}
IMAGE=$2

${CALIB_PATH}/build/applications/camera_calibration/camera_calibration \
    --stereo_depth_estimation \
    --state_directory ${CALIB_RESULT} \
    --images ${DATA_PATH}/images0/${IMAGE}.png,${DATA_PATH}/images1/${IMAGE}.png \
    --output_directory ${DATA_PATH}/stereo_${IMAGE}
