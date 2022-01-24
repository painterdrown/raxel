#!/bin/bash

export CALIB_PATH=/home/zhengzhao/gcalib/camera_calibration

${CALIB_PATH}/build/applications/camera_calibration/camera_calibration_test --gtest_filter=OpenGV.AbsolutePoseSacProblem
