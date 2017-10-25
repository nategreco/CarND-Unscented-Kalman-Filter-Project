#include "ukf.h"
#include "Eigen/Dense"
#include <iostream>
#include <math.h>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

void normalizeAngle (double &angle) {
  while (angle > M_PI) angle =- M_PI;
  while (angle < M_PI) angle =+ M_PI;
  return;
}

/**
 * Initializes Unscented Kalman filter
 */
UKF::UKF() {

  // initially set to false, set to true in first call of ProcessMeasurement
  is_initialized_ = false;

  // if this is false, laser measurements will be ignored (except during init)
  use_laser_ = true;

  // if this is false, radar measurements will be ignored (except during init)
  use_radar_ = false;

  // initial state vector
  x_ = VectorXd(5);
  //Actual initialization occurs in UKF::ProcessMeasurement()

  // initial covariance matrix
  P_ = MatrixXd(5, 5);
  P_ << 1, 0, 0, 0, 0,
        0, 1, 0, 0, 0,
        0, 0, 1, 0, 0,
        0, 0, 0, 1, 0,
        0, 0, 0, 0, 1;

  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = 1.75;

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 0.45;

  // Laser measurement noise standard deviation position1 in m
  std_laspx_ = 0.15;

  // Laser measurement noise standard deviation position2 in m
  std_laspy_ = 0.15;

  // Radar measurement noise standard deviation radius in m
  std_radr_ = 0.3;

  // Radar measurement noise standard deviation angle in rad
  std_radphi_ = 0.03;

  // Radar measurement noise standard deviation radius change in m/s
  std_radrd_ = 0.3;

  // State dimension
  n_x_ = x_.size();

  // Augmented state dimension
  n_aug_ = n_x_ + 2;

  // Sigma point spreading parameter
  n_sig_ = 2 * n_aug_ + 1;

  // Sigma point spreading parameter
  lambda_ = 3 - n_aug_;

  // Weights of sigma points
  weights_ = VectorXd(n_sig_);
  double weight_0 = lambda_ / (lambda_ + n_aug_);
  weights_[0] = weight_0;
  for (int i = 1; i < n_sig_; ++i) {
    double weight = 0.5 / (n_aug_ + lambda_);
    weights_[i] = weight;
  }

  // Lidar noise covariance matrix
  R_lidar_ = MatrixXd(2, 2); 
  R_lidar_ << std_laspx_ * std_laspx_,0,
              0,std_laspy_ * std_laspy_;

  // Radar noise covariance matrix
  R_radar_ = MatrixXd(3, 3); 
  R_radar_ << std_radr_ * std_radr_, 0, 0,
              0, std_radphi_ * std_radphi_, 0,
              0, 0,std_radrd_ * std_radrd_;
}

UKF::~UKF() {}

/**
 * @param {MeasurementPackage} meas_package The latest measurement data of
 * either radar or laser.
 */
void UKF::ProcessMeasurement(MeasurementPackage meas_package) {
  //Check if initialized
  if (!is_initialized_) {
    std::cout << "Initializing UKF..." << std::endl;
    //Use initial measurement for initialization
    if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
      //Convert to cartesian coordinates
      float px = meas_package.raw_measurements_[0] *
        cos(meas_package.raw_measurements_[1]); 
      float py = meas_package.raw_measurements_[0] *
        sin(meas_package.raw_measurements_[1]);
      float vx = meas_package.raw_measurements_[2] *
        cos(meas_package.raw_measurements_[1]);
      float vy = meas_package.raw_measurements_[2] *
        sin(meas_package.raw_measurements_[1]);
      float v  = sqrt(vx*vx + vy*vy);
      x_ << px, py, v, 0, 0;
    } else if (meas_package.sensor_type_ == MeasurementPackage::LASER) {
      //Already cartesian, but due to lack of information assume initial velocties zero
      x_ << meas_package.raw_measurements_[0], meas_package.raw_measurements_[1], 0, 0, 0;
    }
    //Prevent division by zero
    if (fabs(x_[0]) < 0.0001 && fabs(x_[1]) < 0.0001) {
      x_[0] = 0.0001;
      x_[1] = 0.0001;
    }
    //Force initial dt = 0.5s
    time_us_ = meas_package.timestamp_ - (0.05 * 1000000.0);
    //Flag initialized
    is_initialized_ = true;
  }
  
  //Get time elapsed  in seconds
  double dt = (meas_package.timestamp_ - time_us_) / 1000000.0;
  time_us_ = meas_package.timestamp_;
  
  //Predict
  MatrixXd Xsig_pred = Prediction(dt);
  
  //Update
  if (meas_package.sensor_type_ == MeasurementPackage::RADAR && use_radar_) {
      UpdateRadar(meas_package, Xsig_pred);
  }
  if (meas_package.sensor_type_ == MeasurementPackage::LASER && use_laser_) {
      UpdateLidar(meas_package, Xsig_pred);
  }
  std::cout << "Update complete, x_: " << x_ << std::endl;
}

/**
 * Predicts sigma points, the state, and the state covariance matrix.
 * @param {double} delta_t the change in time (in seconds) between the last
 * measurement and this one.
 */
MatrixXd UKF::Prediction(double delta_t) {
  //Generate Sigma Points
  MatrixXd Xsig = MatrixXd(n_x_, n_sig_);
  MatrixXd A = P_.llt().matrixL();
  Xsig.col(0) = x_;
  for (int i = 0; i < n_x_; ++i)
  {
    Xsig.col(i + 1)     = x_ + sqrt(lambda_ + n_x_) * A.col(i);
    Xsig.col(i + 1 + n_x_) = x_ - sqrt(lambda_ + n_x_) * A.col(i);
  }
  
  //Augment Sigma Points
  VectorXd x_aug = VectorXd(n_aug_); //create augmented mean state
  x_aug.fill(0.0);
  x_aug.head(5) = x_;
  MatrixXd P_aug = MatrixXd(n_aug_, n_aug_); //create augmented covariance matrix
  P_aug.fill(0.0);
  P_aug.topLeftCorner(5,5) = P_;
  P_aug(5,5) = std_a_*std_a_;
  P_aug(6,6) = std_yawdd_*std_yawdd_;
  MatrixXd L = P_aug.llt().matrixL(); //create square root matrix
  MatrixXd Xsig_aug = MatrixXd(n_aug_, n_sig_); //create augmented sigma points
  Xsig_aug.col(0)  = x_aug;
  for (int i = 0; i < n_aug_; ++i)
  {
    Xsig_aug.col(i + 1)          = x_aug + sqrt(lambda_ + n_aug_) * L.col(i);
    Xsig_aug.col(i + 1 + n_aug_) = x_aug - sqrt(lambda_ + n_aug_) * L.col(i);
  }
  
  //Predict Sigma Points
  MatrixXd Xsig_pred = MatrixXd(n_x_, n_sig_);
  for (int i = 0; i < n_sig_; ++i) {
    //extract values for better readability
    double p_x = Xsig_aug(0,i);
    double p_y = Xsig_aug(1,i);
    double v = Xsig_aug(2,i);
    double yaw = Xsig_aug(3,i);
    double yawd = Xsig_aug(4,i);
    double nu_a = Xsig_aug(5,i);
    double nu_yawdd = Xsig_aug(6,i);

    //predicted state values
    double px_p, py_p;

    //avoid division by zero
    if (fabs(yawd) > 0.0001) {
        px_p = p_x + v / yawd * ( sin (yaw + yawd * delta_t) - sin(yaw));
        py_p = p_y + v / yawd * ( cos(yaw) - cos(yaw + yawd * delta_t) );
    } else {
        px_p = p_x + v * delta_t * cos(yaw);
        py_p = p_y + v * delta_t * sin(yaw);
    }

    double v_p = v;
    double yaw_p = yaw + yawd*delta_t;
    double yawd_p = yawd;

    //add noise
    px_p += 0.5 * nu_a * delta_t * delta_t * cos(yaw);
    py_p += 0.5 * nu_a * delta_t * delta_t * sin(yaw);
    v_p += nu_a * delta_t;
    yaw_p += 0.5 * nu_yawdd * delta_t * delta_t;
    yawd_p += nu_yawdd * delta_t;

    //write predicted sigma point into right column
    Xsig_pred(0,i) = px_p;
    Xsig_pred(1,i) = py_p;
    Xsig_pred(2,i) = v_p;
    Xsig_pred(3,i) = yaw_p;
    Xsig_pred(4,i) = yawd_p;
  }

  //Predict Mean and Covraiance
  x_.fill(0.0);
  P_.fill(0.0);
  for (int i = 0; i < n_sig_; ++i) {
    x_ = x_ + weights_[i] * Xsig_pred.col(i);
    VectorXd x_diff = Xsig_pred.col(i) - x_;
    //Normalize angle
    normalizeAngle(x_diff[3]);
    P_ = P_ + weights_[i] * x_diff * x_diff.transpose() ;
  }
  
  return Xsig_pred;
}

/**
 * Updates the state and the state covariance matrix using a laser measurement.
 * @param {MeasurementPackage} meas_package
 * @param Xsig_pred Predicted sigma points
 */
void UKF::UpdateLidar(MeasurementPackage meas_package, MatrixXd Xsig_pred) {
  //Predict Measurement
  int n_z = 2;
  MatrixXd Zsig = Xsig_pred.block(0, 0, n_z, n_sig_);

  //Update State
  Update(meas_package, Xsig_pred, Zsig, n_z);
}

/**
 * Updates the state and the state covariance matrix using a radar measurement.
 * @param {MeasurementPackage} meas_package
 * @param Xsig_pred Predicted sigma points
 */
void UKF::UpdateRadar(MeasurementPackage meas_package, MatrixXd Xsig_pred) {
  //Predict Measurement
  int n_z = 3;
  MatrixXd Zsig = MatrixXd(n_z, n_sig_); //transform into measurement space
  for (int i = 0; i < n_sig_; ++i) {
    double p_x = Xsig_pred(0,i);
    double p_y = Xsig_pred(1,i);
    double v  = Xsig_pred(2,i);
    double yaw = Xsig_pred(3,i);
    double v1 = cos(yaw)*v;
    double v2 = sin(yaw)*v;
    Zsig(0,i) = sqrt(p_x*p_x + p_y*p_y);                        //r
    Zsig(1,i) = atan2(p_y,p_x);                                 //phi
    Zsig(2,i) = (p_x*v1 + p_y*v2 ) / sqrt(p_x*p_x + p_y*p_y);   //r_dot
  }

  //Update State
  Update(meas_package, Xsig_pred, Zsig, n_z);
}

/**
 * Common update function called by both UpdateLidar() and UpdateRadar()
 * @param meas_package The measurement at k+1
 * @param Xsig_pred Predicted sigma points
 * @param Zsig The transformed sigma points in measurement space
 * @param n_z The number of measurement components
 */
void UKF::Update(MeasurementPackage meas_package,
                 MatrixXd Xsig_pred,
                 MatrixXd Zsig,
                 int n_z) {
  //Populate z
  VectorXd z = meas_package.raw_measurements_;
  //Mean predicted measurement
  VectorXd z_pred = VectorXd(n_z);
  z_pred = Zsig * weights_;
  
  //calculate cross correlation matrix
  MatrixXd Tc = MatrixXd(n_x_, n_z);
  Tc.fill(0.0);
  for (int i = 0; i < n_sig_; ++i) {
    //residual
    VectorXd z_diff = Zsig.col(i) - z_pred;
    //Normalize angle
    normalizeAngle(z_diff[1]);
    // state difference
    VectorXd x_diff = Xsig_pred.col(i) - x_;
    //Normalize angle
    normalizeAngle(x_diff[3]);
    Tc = Tc + weights_[i] * x_diff * z_diff.transpose();
  }
  //measurement covariance matrix
  MatrixXd S = MatrixXd(n_z, n_z);
  S.fill(0.0);
  for (int i = 0; i < n_sig_; ++i) {
    VectorXd z_diff = Zsig.col(i) - z_pred;
    //Normalize angle
    normalizeAngle(z_diff[1]);
    S = S + weights_[i] * z_diff * z_diff.transpose();
  }
  //add noise covariance matrix
  if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
      S = S + R_radar_;
  } else if (meas_package.sensor_type_ == MeasurementPackage::LASER) {
      S = S + R_lidar_;
  }
  //Kalman gain K;
  MatrixXd K = Tc * S.inverse();
  //residual
  VectorXd z_diff = z - z_pred;
  //Normalize angle
  normalizeAngle(z_diff[1]);
  //update state mean and covariance matrix
  x_ = x_ + K * z_diff;
  P_ = P_ - K * S * K.transpose();
}
