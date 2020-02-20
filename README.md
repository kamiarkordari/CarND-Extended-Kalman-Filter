# Extended Kalman Filter Project
Self-Driving Car Engineer Nanodegree Program

In this project we will utilize a kalman filter to estimate the state of a moving object of interest with noisy lidar and radar measurements.

[image-output]: ./output/kalman_filter.gif "Output"
[![image-output]](https://www.youtube.com/watch?v=8TK2HLlAQHE&feature=youtu.be)


# Dependency
This repository includes two files that can be used to set up and install [uWebSocketIO](https://github.com/uWebSockets/uWebSockets) for either Linux or Mac systems. For windows you can use either Docker, VMware, or even [Windows 10 Bash on Ubuntu](https://www.howtogeek.com/249966/how-to-install-and-use-the-linux-bash-shell-on-windows-10/) to install uWebSocketIO.

Once the install for uWebSocketIO is complete, the main program can be built and run by doing the following from the project top directory.

1. mkdir build
2. cd build
3. cmake ..
4. make
5. ./ExtendedKF


Here is the main protocol that main.cpp uses for uWebSocketIO in communicating with the simulator.

**INPUT**: values provided by the simulator to the c++ program

["sensor_measurement"] => the measurement that the simulator observed (either lidar or radar)


**OUTPUT**: values provided by the c++ program to the simulator

["estimate_x"] <= kalman filter estimated position x

["estimate_y"] <= kalman filter estimated position y

["rmse_x"]

["rmse_y"]

["rmse_vx"]

["rmse_vy"]

---

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
   * On windows, you may need to run: `cmake .. -G "Unix Makefiles" && make`
4. Run it: `./ExtendedKF `

### Project Introduction
In this projet I implemented an extended Kalman filter in C++ using simulated lidar and radar measurements detecting a moving car that travels around our vehicle.

The simulator provides the script the measured data (either lidar or radar), and the script feeds back the measured estimation marker, and RMSE values from its Kalman filter.

### Explanation of the Data File
The github repo contains one data file: `obj_pose-laser-radar-synthetic-input.txt`

The simulator uses this data file, and feed `main.cpp` values from it one line at a time.

Each row represents a sensor measurement where the first column tells if the measurement comes from radar (R) or lidar (L).

For a row containing radar data, the columns are: sensor_type, rho_measured, phi_measured, rhodot_measured, timestamp, x_groundtruth, y_groundtruth, vx_groundtruth, vy_groundtruth, yaw_groundtruth, yawrate_groundtruth.

For a row containing lidar data, the columns are: sensor_type, x_measured, y_measured, timestamp, x_groundtruth, y_groundtruth, vx_groundtruth, vy_groundtruth, yaw_groundtruth, yawrate_groundtruth.

Whereas radar has three measurements (rho, phi, rhodot), lidar has two measurements (x, y).

I use the measurement values and timestamp in your Kalman filter algorithm. Groundtruth, which represents the actual path the bicycle took, is for calculating root mean squared error.

### Reading in the Data
The code to read in and parse the data files is in the main.cpp file. The `main.cpp` file creates instances of a MeasurementPackage.

The code reads in the data file line by line. The measurement data for each line gets pushed onto a measurement_pack_list. The ground truth `[px, py, vx, vy]` for each line in the data file gets pushed onto ground_truth so RMSE can be calculated later from tools.cpp.

### Overview of the Kalman Filter: Initialize, Predict, Update
The three main steps for programming a Kalman filter:

- initializing Kalman filter variables
- predicting where our object is going to be after a time step Δt
- updating where our object is based on sensor measurements

To measure how well the Kalman filter performs, I calculate root mean squared error comparing the Kalman filter results with the provided ground truth.


##### Files in the Github src Folder
The source files are in the src folder of the github repository.

- `main.cpp` - communicates with the simulator receiving data measurements, calls a function to run the Kalman filter, calls a function to calculate RMSE
- `FusionEKF.cpp` - initializes the filter, calls the predict function, calls the update function
- `kalman_filter.cpp` - defines the predict function, the update function for lidar, and the update function for radar
- `tools.cpp`- function to calculate RMSE and the Jacobian matrix


##### How the Files Relate to Each Other
Here is a brief overview of what happens when the code runs:

- `main.cpp` All the main code loops in h.onMessage() and reads in the data and sends a sensor measurement to `FusionEKF.cpp`.
- `FusionEKF.cpp` The measured sensor data is received by calling the ProcessMeasurement() function. `FusionEKF.cpp` has a variable called `ekf_`, which is an instance of a KalmanFilter class. The `ekf_` will hold the matrix and vector values for the filter. `ekf_` instance calls the predict and update equations.
- The KalmanFilter class is defined in `kalman_filter.cpp` and `kalman_filter.h`.

### Final Result
Lidar measurements are red circles, radar measurements are blue circles with an arrow pointing in the direction of the observed angle, and estimation markers are green triangles. The video below shows what the simulator looks like when a c++ script is using its Kalman filter to track the object.

[![image-output]](https://www.youtube.com/watch?v=8TK2HLlAQHE&feature=youtu.be)
