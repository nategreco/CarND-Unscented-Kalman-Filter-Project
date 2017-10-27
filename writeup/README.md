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

Implementation of the Unscented Kalman Filter utilized most code from the lesseons adapted for this implementation.  Some tuning was required, specifically for the values of yaw acceleration standard deviation and longitudinal acceleration standard deviation, otherwise all other provided standard deviation values were used as-is.  Chose values for yaw acceleration and longitudinal acceleration standard deviation were 0.45 and 1.75 respectively.

As recommended in the tips classroom section, special care was taken to prevent division by zero in [1](../src/ukf.cpp#L123), [2](../src/ukf.cpp#L190), and [3](../src/ukf.cpp#L266) locations, as well as normalizing of all angles was managed by [normalizeAngle](../src/ukf.cpp#L13) in [1](../src/ukf.cpp#L227), [2](../src/ukf.cpp#L300), [3](../src/ukf.cpp#L306), [4](../src/ukf.cpp#L317), and [5](../src/ukf.cpp#L333) locations. The normalization function ensured that all angles remaiend within the region of -pi to pi.


#### 2. Results

Upon inspection of the first data set, one can clearly see that the result position is much cleaner and smoother than the incoming sensors and the noise is greatly reduced.  For data set one, typical RMSE values for x, y, vx, and vy, were 0.07, 0.08, 0.33, and 0.23 respectively:

Dataset 1:

![Dataset 1][image1]

Data set 2 provided similar results, with typical RMSE values for x, y, vx, and vy, of 0.07, 0.07, 0.62, and 0.28 respectively:

Dataset 2:

![Dataset 2][image2]
