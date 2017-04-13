#include "kalman_filter.h"
#include "tools.h"
#define M_PI 3.14159265358979323846

using Eigen::MatrixXd;
using Eigen::VectorXd;

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
  DONE:
    * predict the state
  */
  x_ = F_ * x_;
  MatrixXd Ft = F_.transpose();
  P_ = F_ * P_ * Ft + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
  DONE:
    * update the state by using Kalman Filter equations
  */
  VectorXd z_pred = H_ * x_;
  VectorXd y = z - z_pred;
  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd Si = S.inverse();
  MatrixXd PHt = P_ * Ht;
  MatrixXd K = PHt * Si;

  //new estimate
  x_ = x_ + (K * y);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;

}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
  DONE:
    * update the state by using Extended Kalman Filter equations
  */

  Tools tools;
  MatrixXd Hj = MatrixXd(3, 4);
  Hj = tools.CalculateJacobian(x_);

  VectorXd r_out = VectorXd(3);
  double ro = sqrt(x_[0]*x_[0] + x_[1]*x_[1]);
  double phi = atan(x_[1]/x_[0]);
  if (phi<-M_PI)
      phi += 2*M_PI;
  else if (phi>M_PI)
      phi -= 2*M_PI;
  double ro_dot = ((x_[0]*x_[2])+(x_[1]*x_[3]))/ro;

  r_out << ro, phi, ro_dot;


  VectorXd z_pred = r_out;
  VectorXd y = z - z_pred;
  MatrixXd Ht = Hj.transpose();
  MatrixXd S = Hj * P_ * Ht + R_;
  MatrixXd Si = S.inverse();
  MatrixXd PHt = P_ * Ht;
  MatrixXd K = PHt * Si;

  //new estimate
  x_ = x_ + (K * y);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * Hj) * P_;

}
