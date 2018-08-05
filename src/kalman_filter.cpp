#include "kalman_filter.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

// Please note that the Eigen library does not initialize 
// VectorXd or MatrixXd objects with zeros upon creation.
VectorXd h(VectorXd); // function h(x)

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
  TODO:
    * predict the state
  */
    x_ = F_ * x_;
    MatrixXd Ft = F_.transpose();
    P_ = F_ * P_ * Ft + Q_;
}

// function h(x)
VectorXd h(VectorXd pred_state) {
    double px = pred_state[0];
    double py = pred_state[1];
    double vx = pred_state[2];
    double vy = pred_state[3];
    
    VectorXd proj_state = VectorXd(3);
    proj_state << 0,0,0;
    
    if(px == 0 && py == 0) {
        return proj_state;
    }
    
    double rho = sqrt(px*px + py*py);
    double phi = atan2(py, px);
    double rho_dot = ((px * vx) + (py * vy))/rho;
    
    proj_state[0] = rho;
    proj_state[1] = phi;
    proj_state[2] = rho_dot;
    
    return proj_state;
    
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
  TODO:
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
  TODO:
    * update the state by using Extended Kalman Filter equations
  */
    VectorXd z_pred = h(x_); // function h(x)
    VectorXd y = z - z_pred;
    
    // check angle -PI < phi < PI
    if(y[1] > M_PI){
        y[1] = y[1] - 2*M_PI;
    } else if (y[1] < -M_PI){
        y[1] = y[1] + 2*M_PI;
    }
    
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
