#include "ukf.h"
#include "Eigen/Dense"
#include <iostream>

#include <time.h>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;


static double NormalizeAngle(double angle)
{
  return angle - (2*M_PI) * floor((angle + M_PI) / (2 * M_PI));
}

/**
 * Initialize Unscented Kalman filter
 */
UKF::UKF() {
  // if this is false, laser measurements will be ignored (except during init)
  use_laser_ = true;

  // if this is false, radar measurements will be ignored (except during init)
  use_radar_ = true;

  // initial state vector
  x_ = VectorXd(5);

  // initial covariance matrix
  P_ = MatrixXd(5, 5);

  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = 30;

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 30;

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

  is_initialized_ = false;
  time_us_ = 0;

  // CRTV model used
  n_x_ = 5;
  n_aug_ = 7;

  // UKF free parameter
  lambda_ = 3 - n_x_;

  // Nb sigma points: 2*n_aug_+1
  Xsig_pred_ = MatrixXd(n_x_, 2*n_aug_+1);
  Xsig_pred_.fill(0.0);
  weights_ = VectorXd(2*n_aug_+1);

  //set weights
  weights_.fill(1.0);
  weights_ /= (2*(lambda_ + n_aug_));
  weights_(0) = lambda_ / (lambda_ + n_aug_);

  // Based on ground truth values analysis: cf data/get_sigma.m
  //std_a_ = 0.14;
  //std_yawdd_ = 0.2;

  // Rule of thumb: std_a set to ~ 1/2 a_max for an object
  // car in urban environment: std_a = 3 m/s^2 
  // for a bicicle, choose a lower value: std_a = 1 m/s^2
  std_a_ = 1; // std for linear acceleration of object tracked
  std_yawdd_ = 0.5; // std for angular acceleration of object tracked

  // huge value, inconsistent at start
  NIS_laser_ = 1000;
  NIS_radar_ = 1000;

  R_radar_ = MatrixXd(3,3);
  R_radar_ << std_radr_ * std_radr_, 0, 0,
              0, std_radphi_ * std_radphi_, 0,
              0, 0, std_radrd_ * std_radrd_;

  R_laser_ = MatrixXd(2, 2);
  R_laser_ << std_laspx_ * std_laspx_, 0,
              0, std_laspy_ * std_laspy_;

}

UKF::~UKF() {}

/**
 * @param {MeasurementPackage} meas_package The latest measurement data of
 * either radar or laser.
 */
void UKF::ProcessMeasurement(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Make sure you switch between lidar and radar
  measurements.
  */


  /*****************************************************************************
   *  Initialization
   ****************************************************************************/
  if (!is_initialized_) {
    /**
      * Initialize the state x_ with the first measurement.
      * Create the covariance matrix.
    */
    // first measurement

    if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
      /**
      Convert radar from polar to cartesian coordinates and initialize state.
      */
      double rho = meas_package.raw_measurements_[0];
      double phi = meas_package.raw_measurements_[1];
      x_ <<  rho * cos(phi), rho * sin(phi), 0, 0, 0;

      P_ << 1, 0, 0, 0, 0,
            0, 1, 0, 0, 0,
            0, 0, 1, 0, 0,
            0, 0, 0, 0.5, 0,
            0, 0, 0, 0, 0.5;


      // done initializing, no need to predict or update
      time_us_ = meas_package.timestamp_;
      is_initialized_ = true;
    }
    else if (meas_package.sensor_type_ == MeasurementPackage::LASER) {
      /**
      Initialize state.
      */
      //set the state with the initial location and zero velocity
      x_ << meas_package.raw_measurements_[0], meas_package.raw_measurements_[1], 0, 0, 0;

      P_ << 1, 0, 0, 0, 0,
            0, 1, 0, 0, 0,
            0, 0, 1, 0, 0,
            0, 0, 0, 0.5, 0,
            0, 0, 0, 0, 0.5;


      // done initializing, no need to predict or update
      time_us_ = meas_package.timestamp_;
      is_initialized_ = true;
    }

    return;
  }

  clock_t start = clock();

  /*****************************************************************************
   *  Prediction
   ****************************************************************************/

  //compute the time elapsed between the current and previous measurements
  double dt = (meas_package.timestamp_ - time_us_) / 1000000.0; //dt in seconds
  time_us_ = meas_package.timestamp_;

  Prediction(dt);

  /*****************************************************************************
   *  Update
   ****************************************************************************/

  if (meas_package.sensor_type_ == MeasurementPackage::RADAR && use_radar_) {
    // Radar updates
    UpdateRadar(meas_package);
  } else if (meas_package.sensor_type_ == MeasurementPackage::LASER && use_laser_) {
    // Laser updates
    UpdateLidar(meas_package);
  }

  clock_t stop = clock();
  double elapsed = (double)(stop - start) * 1000000.0 / CLOCKS_PER_SEC;
  cout << "Process time in us: " << elapsed << endl;

  // print the output
  cout << "x_ = " << x_ << endl;
  cout << "P_ = " << P_ << endl;
}

/**
 * Predicts sigma points, the state, and the state covariance matrix.
 * @param {double} delta_t the change in time (in seconds) between the last
 * measurement and this one.
 */
void UKF::Prediction(double dt) {
  /**
  TODO:

  Complete this function! Estimate the object's location. Modify the state
  vector, x_. Predict sigma points, the state, and the state covariance matrix.
  */

  //cout << "dt : " << dt << endl;

  /*-------------------------------------------------------
   1) Create Augmented State, Augmented Covariance matrix
  --------------------------------------------------------*/

  //create augmented mean vector
  VectorXd x_aug = VectorXd(n_aug_);
  //create augmented state covariance
  MatrixXd P_aug = MatrixXd(n_aug_, n_aug_);

  x_aug.fill(0.0);
  x_aug.head(n_x_) = x_;
  // mean of last 2 elements, noise values, is 0

  P_aug.fill(0.0);
  P_aug.topLeftCorner(n_x_, n_x_) = P_;
  P_aug(n_x_, n_x_) = std_a_ * std_a_;
  P_aug(n_x_+1, n_x_+1) = std_yawdd_ * std_yawdd_;

  /*------------------------------------------------------
   2) Select Augmented Sigma points (Xsig_aug)
  ------------------------------------------------------*/

  // number of sigma points
  int n_sig = 2*n_aug_+1;
  //create sigma point matrix
  MatrixXd Xsig_aug = MatrixXd(n_aug_, n_sig);
  Xsig_aug.fill(0.0);

  //create square root matrix (A_aug)
  MatrixXd A_aug = P_aug.llt().matrixL();

  //create augmented sigma points
  double c1 = sqrt(lambda_ + n_aug_);
  Xsig_aug.middleCols(1, n_aug_) = c1 * A_aug;
  Xsig_aug.middleCols(n_aug_+1, n_aug_) = -c1 *A_aug;
  Xsig_aug += x_aug.replicate(1, n_sig);

  /*------------------------------------------------------------
   3) Apply non linear CRTV process motion model: dt dependant
  -------------------------------------------------------------*/

  // This implements the CTRV process model
  //predict sigma points
  for (int n = 0; n < n_sig; n++) {
    const double px = Xsig_aug(0, n);
    const double py = Xsig_aug(1, n);
    const double v = Xsig_aug(2, n);
    const double psi = Xsig_aug(3, n);
    const double psi_dot = Xsig_aug(4, n);

    const double nu_a = Xsig_aug(5, n);
    const double nu_yawdd = Xsig_aug(6, n);

    //avoid division by zero
    //write predicted sigma points into right column

    if (fabs(psi_dot) > 0.001) {
      Xsig_pred_(0, n) = px + (v/psi_dot)*(sin(psi+psi_dot*dt)-sin(psi));
      Xsig_pred_(1, n) = py + (v/psi_dot)*(-cos(psi+psi_dot*dt)+cos(psi));
    } else {
      Xsig_pred_(0, n) = px + v*cos(psi)*dt; 
      Xsig_pred_(1, n) = py + v*sin(psi)*dt; 
    }

    Xsig_pred_(2, n) = v + 0;
    Xsig_pred_(3, n) = psi + psi_dot*dt;
    Xsig_pred_(4, n) = psi_dot + 0;

    Xsig_pred_(0, n) += (0.5*dt*dt*cos(psi)*nu_a);
    Xsig_pred_(1, n) += (0.5*dt*dt*sin(psi)*nu_a);
    Xsig_pred_(2, n) += (dt*nu_a);
    Xsig_pred_(3, n) += (0.5*dt*dt*nu_yawdd);
    Xsig_pred_(4, n) += (dt*nu_yawdd);
  }  

  /*----------------------------------------------
   4) Predict new Mean and new Covariance
  ----------------------------------------------*/

  //create vector for predicted state
  VectorXd x = VectorXd(n_x_);
  //create covariance matrix for prediction
  MatrixXd P = MatrixXd(n_x_, n_x_);

  //predict state mean
  x.fill(0.0);
  for (int i = 0; i < n_sig; i++) {
    x = x + weights_(i) * Xsig_pred_.col(i); 
  }

  //predict state covariance matrix
  P.fill(0.0);
  for (int i = 0; i < n_sig; i++) {
    VectorXd xi = Xsig_pred_.col(i) - x; 

    //angle normalization
    //while (xi(3)> M_PI) xi(3)-=2.*M_PI;
    //while (xi(3)<-M_PI) xi(3)+=2.*M_PI;
    xi(3) = NormalizeAngle(xi(3));

    P = P + weights_(i) * xi * xi.transpose();
  }

  x_ = x;
  P_ = P;
}

/**
 * Updates the state and the state covariance matrix using a laser measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateLidar(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Use lidar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the lidar NIS.
  */


  //set measurement dimension, lidar can measure px, py
  int n_z = 2;
  // Number of sigma points
  int n_sig = 2*n_aug_+1;

  /*-------------------------------------------------------------------------------------
   1) Apply non linear measurement model on Sigma points (we got from Prediction step)
  -------------------------------------------------------------------------------------*/

  //create matrix for sigma points in measurement space
  MatrixXd Zsig = MatrixXd(n_z, n_sig);

  //transform sigma points into measurement space
  for (int n = 0; n < n_sig; n++) {
    double px = Xsig_pred_(0, n);
    double py = Xsig_pred_(1, n);
    //double v = Xsig_pred_(2, n);
    //double psi = Xsig_pred_(3, n);
    //double psi_dot = Xsig_pred_(4, n);

    Zsig(0, n) = px;
    Zsig(1, n) = py;
  }

  /*--------------------------------------------------
   2) Calculate Mean and Covariance: z_pred and S
  ---------------------------------------------------*/

  //mean predicted measurement (z_pred)
  VectorXd z_pred = VectorXd(n_z);

  //measurement covariance matrix (S)
  MatrixXd S = MatrixXd(n_z, n_z);

  //calculate mean predicted measurement (z_pred)
  z_pred.fill(0.0);
  for (int i = 0; i < n_sig; i++) {
    z_pred = z_pred + weights_(i) * Zsig.col(i); 
  }

  //calculate measurement covariance matrix (S)
  S.fill(0.0);
  for (int i = 0; i < n_sig; i++) {
    VectorXd zi = Zsig.col(i) - z_pred; 
    S = S + weights_(i) * zi * zi.transpose();
  }

  S = S + R_laser_;

  /*-------------------------------------------------------------------------------------------
   3) Calculate Cross-Correlation between Sigma points in state space and measurement space
  -------------------------------------------------------------------------------------------*/

  // matrix for cross correlation (Tc)
  MatrixXd Tc = MatrixXd(n_x_, n_z);

  //calculate Tc
  Tc.fill(0.0);
  for (int i = 0; i < n_sig; i++) {
    VectorXd xi = Xsig_pred_.col(i) - x_; 
    VectorXd zi = Zsig.col(i) - z_pred; 
    Tc = Tc + weights_(i) * xi * zi.transpose();
  }

  /*---------------------------------------------------
   4) Calculate Kalman gain K
  ---------------------------------------------------*/
  MatrixXd K = Tc * S.inverse();

  /*-------------------------------------------------
   5) Udpate state Mean and Covariance matrix
  --------------------------------------------------*/
  VectorXd z = meas_package.raw_measurements_;

  VectorXd z_diff = z - z_pred;

  x_ = x_ + K * z_diff;
  P_ = P_ - K * S * K.transpose();

  /*------------------------------------------------------------------------------------------------------
   6) Calculate NIS value for consistency check: 95% of values should be below 5.9 (2 Degrees of Freedom)
  -------------------------------------------------------------------------------------------------------*/
  VectorXd error = z - z_pred; 
  // NIS value follows chi-squared distribution
  NIS_laser_ = error.transpose() * S.inverse() * error;
  cout << "NIS_laser_ = " << NIS_laser_ << endl;
}

/**
 * Updates the state and the state covariance matrix using a radar measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateRadar(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Use radar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the radar NIS.
  */

  //set measurement dimension, radar can measure r, phi, and r_dot
  int n_z = 3;
  // Number of sigma points
  int n_sig = 2*n_aug_+1;

  /*------------------------------------------------------------------------------------
   1) Apply non linear measurement model on Sigma points (calculated in Prediction step)
  -------------------------------------------------------------------------------------*/

  //create matrix for sigma points in measurement space (Zsig)
  MatrixXd Zsig = MatrixXd(n_z, n_sig);

  //transform sigma points into measurement space (n_sig)
  for (int n = 0; n < n_sig; n++) {
    double px = Xsig_pred_(0, n);
    double py = Xsig_pred_(1, n);
    double v = Xsig_pred_(2, n);
    double psi = Xsig_pred_(3, n);
    //double psi_dot = Xsig_pred_(4, n);


    // avoid divide by 0: px, py in meters ...
    if (fabs(px) < 0.001) px = 0.001;
    if (fabs(py) < 0.001) py = 0.001;

    double rho = sqrt(px*px+py*py);
    double phi = atan2(py, px);
    double rho_dot = (px*cos(psi)*v+py*sin(psi)*v) / rho;

    Zsig(0, n) = rho;
    Zsig(1, n) = phi;
    Zsig(2, n) = rho_dot;
  }

  /*------------------------------------------------
   2) Calculate Mean (z_pred) and Covariance (S)
  --------------------------------------------------*/

  // Mean Predicted Measurement (z_pred)
  VectorXd z_pred = VectorXd(n_z);
  
  //measurement Covariance matrix (S) 
  MatrixXd S = MatrixXd(n_z, n_z);

  //calculate mean predicted measurement
  z_pred.fill(0.0);
  for (int i = 0; i < n_sig; i++) {
    z_pred = z_pred + weights_(i) * Zsig.col(i); 
  }

  //calculate measurement covariance matrix S
  S.fill(0.0);
  for (int i = 0; i < n_sig; i++) {
    VectorXd zi = Zsig.col(i) - z_pred; 

    //angle normalization
    //while (zi(1)> M_PI) zi(1)-=2.*M_PI;
    //while (zi(1)<-M_PI) zi(1)+=2.*M_PI;
    zi(1) = NormalizeAngle(zi(1));

    S = S + weights_(i) * zi * zi.transpose();
  }

  S = S + R_radar_;

  /*-------------------------------------------------------------------------------------------
   3) Calculate Cross-Correlation between Sigma points in state space and measurement space
  -------------------------------------------------------------------------------------------*/

  //create matrix for cross correlation Tc
  MatrixXd Tc = MatrixXd(n_x_, n_z);

  //calculate cross correlation matrix
  Tc.fill(0.0);
  for (int i = 0; i < n_sig; i++) {
    VectorXd xi = Xsig_pred_.col(i) - x_; 
    VectorXd zi = Zsig.col(i) - z_pred; 

    //angle normalization
    //while (zi(1)> M_PI) zi(1)-=2.*M_PI;
    //while (zi(1)<-M_PI) zi(1)+=2.*M_PI;
    zi(1) = NormalizeAngle(zi(1));

    //angle normalization
    //while (xi(3)> M_PI) xi(3)-=2.*M_PI;
    //while (xi(3)<-M_PI) xi(3)+=2.*M_PI;
    xi(3) = NormalizeAngle(xi(3));

    Tc = Tc + weights_(i) * xi * zi.transpose();
  }

  /*---------------------------------------------------
   4) Calculate Kalman Gain (K)
  ----------------------------------------------------*/
  MatrixXd K = Tc * S.inverse();

  /*--------------------------------------------------
   5) Udpate state Mean (x_) and Covariance matrix (P_)
  ---------------------------------------------------*/
  VectorXd z = meas_package.raw_measurements_;
  VectorXd z_diff = z - z_pred;
  while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
  while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;

  x_ = x_ + K * z_diff;
  P_ = P_ - K * S * K.transpose();

  /*-------------------------------------------------------------------------------------------------------
   6) Calculate NIS value for consistency check: 95% of values should be below 7.8 (3 Degrees of Freedom)
  --------------------------------------------------------------------------------------------------------*/
  VectorXd error = z - z_pred;

  // NIS value follows chi-squared distribution
  NIS_radar_ = error.transpose() * S.inverse() * error;
  cout << "NIS_radar_ = " << NIS_radar_ << endl;
}
