#include "kalman_filter.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}


/** Initialization */

void KalmanFilter::Init(
                        VectorXd &x_in,
                        MatrixXd &P_in,
                        MatrixXd &F_in,
                        MatrixXd &H_in,
                        MatrixXd &R_in,
                        MatrixXd &Q_in) {
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  R_ = R_in;
  Q_ = Q_in;
}

/** Predict the state */

void KalmanFilter::Predict() {
  x_ = F_ * x_;
  MatrixXd Ft = F_.transpose();
  P_ = F_ * P_ * Ft + Q_;
}

/** Update the state by using Kalman Filter equations */

void KalmanFilter::Update(const VectorXd& z,
                          const MatrixXd& H,
                          const MatrixXd& R) {

  const VectorXd z_pred = H_ * x_;
  UpdateEKF(z, z_pred, H, R);
  
}


/** Update the state by using Extended Kalman Filter equations (with already predicted measurements) */

void KalmanFilter::UpdateEKF(const VectorXd& z,
                             const VectorXd& z_pred,
                             const MatrixXd& H,
                             const MatrixXd& R) {

  /*
  float rho = sqrt(x_(0)*x_(0) + x_(1)*x_(1));
  float phi = atan2(x_(1), x_(0));
  float rho_dot;
  if (fabs(rho) < 0.0001) {
    rho_dot = 0;
  } else {
    rho_dot = (x_(0)*x_(2) + x_(1)*x_(3))/rho;
  }
  VectorXd z_pred(3);
  z_pred << rho, phi, rho_dot; 
  */
  
  const VectorXd y = z - z_pred;
  const MatrixXd Ht = H_.transpose();
  const MatrixXd S = H_ * P_ * Ht + R_;
  const MatrixXd Si = S.inverse();
  const MatrixXd K = P_ * Ht * Si;
  
  //new estimate
  x_ = x_ + (K * y);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;
}
