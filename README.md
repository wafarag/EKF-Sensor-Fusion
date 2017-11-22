# Sensor Fusion using Extended Kalman Filter

<img src="EKF_design.png" width="700" alt="Combined Image" />

The main goal of the project is to apply Extended Kalman Filter to fuse data from LiDAR and Radar sensors mounted on a self-driving vehicle (noisy environment). The pipeline is implemented using C++. 

For more details, please, check out the [source code directory](https://github.com/wafarag/EKF-Sensor-Fusion/tree/master/src) and the [testing data file](https://github.com/wafarag/EKF-Sensor-Fusion/blob/master/data/obj_pose-laser-radar-synthetic-input.txt). 

## Content of this repo
- `scr` a directory with the project code:
  - `main.cpp` - reads in data, calls a function to run the Kalman filter, calls a function to calculate RMSE
  - `FusionEKF.cpp` - initializes the filter, calls the predict function, calls the update function
  - `kalman_filter.cpp`- defines the predict function, the update function for lidar, and the update function for radar
  - `tools.cpp` - a function to calculate RMSE and the Jacobian matrix
- `data`  a directory with two input files, provided by Udacity
- `results`  a directory with output and log files
- `Docs` a directory with files formats description

## Result

The results of the EKF fusion algorithm for:
* Both LiDAR and RADAR: RMS => [0.095871,  0.0930713,  0.295331,  0.37495].
* For LiDAR alone: RMS => [0.181518,  0.15348,  0.474065,  0.468911]. 
* For RADAR alone: RMS => [10.9225, 7.92431, 7.80318, 6.49818].
