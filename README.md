# Raxel: Ray-pixel Camera Calibration

A well-rounded camera calibration pipeline (in progress)  for central ray-pixel imaging models.

## Requirements

+ [camera_calibration](https://github.com/puzzlepaint/camera_calibration)
+ OpenCV 4.0.0

## Calibration

```sh
# extract corners
./script/extract_feature.sh <data_path>

# calibrate
./script/calibrate.sh <data_path>
./script/calibrate.sh --sector <data_path>  # Using sector grid
```

## Image Undistortion and Stereo Rectification

```sh
mkdir build
cd build
cmake ..
make
```

Refer to [main.cc](./src/main.cc) for more details.
