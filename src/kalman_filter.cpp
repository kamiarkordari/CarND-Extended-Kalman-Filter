#include "kalman_filter.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

/*
 * Please note that the Eigen library does not initialize
 *   VectorXd or MatrixXd objects with zeros upon creation.
 */

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  R_ = R_in;
  Q_ = Q_in;
}

void KalmanFilter::Predict() {
  /**
   * TODO: predict the state
   */
   x = F_ * x_; //
   MatrixXd Ft = F.transpose(); //
   P_ = F_ * P * Ft + Q_; //

}

void KalmanFilter::Update(const VectorXd &z) {
  /**
   * TODO: update the state by using Kalman Filter equations
   */
   VectorXd y = z - (H * x);
   MatrixXd S = H * P * H.transpose() + R;
   MatrixXd K = P * H.transpose() * S.inverse();

   x = x + (K * y);
   P = (I - (K * H)) * P;

}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
   * TODO: update the state by using Extended Kalman Filter equations
   */

   float x = ekf_.x_(0);
   float y = ekf_x.x_(1);
   float vx = ekf_.x_(2);
   float vy = ekf_.x_(3);

   float rho = sqrt(x*x+y*y);
   float thera = atan2(y,x);
   float rho_dot = (x*vx+y*vy)/rho;
   VectorXd z_pred = VectorXd(3);
   z_pred << rho,theta,rho_dot;

   VectorXd y = z - z_pred;

   MatrixXd Ht = H.transpose();
   MatrixXd S = H * P * Ht + R;
   MatrixXd Si = S.inverse();
   MatrixXd K = P * Ht * Si;

   x = x + (K * y); //
   P = (I - (K * H)) * P; //


}
