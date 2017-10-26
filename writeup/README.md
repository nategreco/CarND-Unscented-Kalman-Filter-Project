# **Unscented Kalman Filters**

### Radar and Lidar sensor fusion utilizing unscented kalman filters

---

**Unscented Kalman Filters Project**

The goals / steps of this project are the following:

* Complete starter code to succesfully fuse lidar and radar sensor data with an unscented kalman filter


[//]: # (Image References)
[image1]: ./Dataset1.png "Results 1"
[image2]: ./Dataset2.png "Results 2"

---

### Writeup / README

#### 1. Provide a Writeup / README that includes all the rubric points and how you addressed each one.

My project includes the following files:
* [main.cpp](../src/main.cpp) main program that runs the IO server (given from starter code)
* [ukf.cpp](../src/ukf.cpp) contains the unscented kalman filter implementation
* [ukf.h](../src/ukf.h) header file for [ukf.cpp](../src/ukf.cpp)
* [tools.cpp](../src/tools.cpp) contains RMSE calculation
* [Dataset1.png](./Dataset1.png) result image from dataset 1
* [Dataset2.png](./Dataset2.png) result image from dataset 2

### Discussion

#### 1. Implementation




#### 2. Results

Upon inspection of the first data set, one can clearly see that the result position is much cleaner and smoother than the incoming sensors and the nosie is greatly reduced:

Dataset 1:

![Dataset 1][image1]

Data set 2 provided similar results:

Dataset 2:

![Dataset 2][image2]
