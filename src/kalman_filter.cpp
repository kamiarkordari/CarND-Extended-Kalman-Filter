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
   x_ = F_ * x_; //
   MatrixXd Ft = F.transpose(); //
   P_ = F_ * P * Ft + Q_; //

}

void KalmanFilter::Update(const VectorXd &z) {
  /**
   * TODO: update the state by using Kalman Filter equations
   */
   VectorXd y = z - (H * _x);
   MatrixXd S = H * P * H.transpose() + R;
   MatrixXd K = P * H.transpose() * S.inverse();

   x_ = x_ + (K * y);
   P_ = (I - (K * H)) * P_;

}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
   * TODO: update the state by using Extended Kalman Filter equations
   */

   float px = ekf_.x_(0);
   float py = ekf_x.x_(1);
   float vx = ekf_.x_(2);
   float vy = ekf_.x_(3);

   float ro = sqrt(px*px + py*py);
   float theta = atan2(py, px);
   float ro_dot = (px*vx + py*vy)/ro;

   // check division by zero
   if (ro < 0.0001) {
     cout << "CalculateJacobian () - Error - Division by Zero" << endl;
     return;
   }

   VectorXd z_pred = VectorXd(3);
   z_pred << ro,theta,ro_dot;

   VectorXd y = z - z_pred;

   // make sure the angel is between -pi and pi
   // [...]

   MatrixXd Ht = H_.transpose();
   MatrixXd S = H_ * P_ * Ht + R_;
   MatrixXd Si = S.inverse();
   MatrixXd K = P_ * Ht * Si;

   x_ = x_ + (K * y); //
   P_ = (I - (K * H_)) * P_; //


}
