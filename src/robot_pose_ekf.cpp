/*
 * @Description: optical flow, imu, encoder融合定位
 * @Author: Ji Jiawen
 * @Date: 2020-04-13
 */

#include "robot_pose_ekf.h"

RobotPoseEKF::RobotPoseEKF(double b, double kl, double kr, double k, 
                  double l, double psi, double phi, double ksx, double ksy) : 
                  b_(b), kl_(kl), kr_(kr), k_(k), l_(l), psi_(psi),
                  phi_(phi), ksx_(ksx), ksy_(ksy), is_inited_(false) {
  // initial pose is (0, 0, 0), convariance is zero matrix
  state_.setZero();
  sigma_.setZero();
  last_u_.setZero();
}

void RobotPoseEKF::AddEncoderData(unsigned int time, const int32_t& enc_l, 
                                const int32_t& enc_r, const int32_t& theta) {
  if (!is_inited_) {
    last_time_ = time;
    last_enc_l_ = enc_l;
    last_enc_r_ = enc_r;
    last_theta_ = theta;    
    is_inited_ = true;
    return;
  }
  
  // 计算时间差单位秒
  double dt = (time - last_time_) / 1000.;

  // 计算左右轮位移距离
  int32_t delta_enc_l = enc_l - last_enc_l_;
  int32_t delta_enc_r = enc_r - last_enc_r_;
  // 防止溢出
  if (abs(delta_enc_l) > 100)
    delta_enc_l = 0;
  if (abs(delta_enc_r) > 100)
    delta_enc_r = 0;
  double delta_s_l = kl_ * delta_enc_l;
  double delta_s_r = kr_ * delta_enc_r;

  // 计算位姿变化量
  // double d_theta_j = (delta_s_r - delta_s_l) / 1000.0 / b_; 
  double d_theta_j = (theta - last_theta_) / 1000.0; // 弧度
  double d_s_j = 0.5 * (delta_s_r + delta_s_l) / 1000.0; // 米

  // TODO:根据打滑情况，将码盘角度增量与陀螺仪增量做互补滤波

  // 系统矩阵
  Eigen::Matrix<double, 6, 6> A;
  A.setZero();
  A(0, 3) = 1.0;
  A(1, 4) = 1.0;
  A(2, 5) = 1.0;
  A(3, 3) = 1.0;
  A(4, 4) = 1.0;
  A(5, 5) = 1.0;

  // 输入矩阵
  Eigen::Matrix<double, 6, 4> B;
  B.setZero(); 
  double theta_j = state_(2); // 当前时刻机器人朝向
  double theta_i = state_(5); // 上一时刻机器人朝向
  double d_s_i = last_u_(0); // 上一时刻真实的控制输入的平移
  double d_theta_i = last_u_(1); // 上一时刻真实的控制输入的旋转
  B(0, 0) = cos(theta_j + 0.5 * d_theta_j);
  B(0, 2) = cos(theta_i + 0.5 * d_theta_i);
  B(1, 0) = sin(theta_j + 0.5 * d_theta_j);
  B(1, 2) = sin(theta_i + 0.5 * d_theta_i);
  B(2, 1) = 1.0;
  B(2, 3) = 1.0;
  B(3, 2) = cos(theta_i + 0.5 * d_theta_i);
  B(4, 2) = sin(theta_i + 0.5 * d_theta_i);
  B(5, 3) = 1.0;

  // 控制输入
  Eigen::Matrix<double, 4, 1> u;
  u << d_s_j, d_theta_j, d_s_i, d_theta_i;

  // 根据系统模型和控制输入更新先验
  mutex_.lock();
  state_ = A * state_ + B * u;
  NormAngle(state_(2));
  NormAngle(state_(5));
  mutex_.unlock();

  // 系统协方差
  Eigen::Matrix<double, 6, 6> Q;
  Q.setZero();
  Q(0, 0) = 0.002; 
  Q(1, 1) = 0.002;
  Q(2, 2) = 0.002;
  Q(3, 3) = 0.002;
  Q(4, 4) = 0.002;
  Q(5, 5) = 0.002;

  // 输入协方差 
  // 正常跑: k_ = 0.02 
  // 防止打滑: k_ = 200.0 
  Eigen::Matrix4d sigma_u;
  sigma_u.setZero();
  sigma_u(0, 0) = k_ * k_ * d_s_j * d_s_j;
  sigma_u(1, 1) = (0.2 * 0.0174533)*(0.2 * 0.0174533)*dt*dt;
  sigma_u(2, 2) = k_ * k_ * d_s_i * d_s_i;
  sigma_u(3, 3) = (0.2 * 0.0174533)*(0.2 * 0.0174533)*dt*dt;

  // 更新协方差
  sigma_ = A * sigma_ * A.transpose() + B * sigma_u * B.transpose() + Q;

  // 保存上一次数据
  last_enc_l_ = enc_l;
  last_enc_r_ = enc_r;
  last_theta_ = theta;
  last_time_ = time;
}

void RobotPoseEKF::OpticalFlowUpdate(const double& sx, const double& sy) {
  if (!is_inited_)
    return;

  if (sx == 0.0 && sy == 0.0)
    return;

  mutex_.lock();
  double theta = state_(2);
  mutex_.unlock();
  double phi_tmp = theta + phi_;
  double sin_phi_tmp = sin(phi_tmp);
  double cos_phi_tmp = cos(phi_tmp);

  double phi_psi = phi_ - psi_;
  double sin_phi_psi = sin(phi_psi);
  double cos_phi_psi = cos(phi_psi);
  
  // 测量矩阵
  Eigen::Matrix<double, 2, 6> H;
  H << sin_phi_tmp, -cos_phi_tmp, l_ * sin_phi_psi, -sin_phi_tmp, cos_phi_tmp, -l_ * sin_phi_psi,
       cos_phi_tmp, sin_phi_tmp, l_ * cos_phi_psi, -cos_phi_tmp, -sin_phi_tmp, -l_ * cos_phi_psi;

  // 光流传感器的测量协方差
  // 正常跑：ksx = 200.0 , ksy = 200.0　
  // 防止打滑：ksx = 0.001 , ksy = 0.001　
  Eigen::Matrix2d R; 
  R << ksx_, 0.0,
        0.0, ksy_;

  // kalman gain
  const Eigen::MatrixXd& P = sigma_;
  Eigen::MatrixXd K = P * H.transpose() * (H * P * H.transpose() + R).inverse();

  // 后验/测量更新
  mutex_.lock();
  state_ = state_ + K * (H * state_ - Eigen::Vector2d(sx, sy));
  NormAngle(state_(2));
  NormAngle(state_(5));
  mutex_.unlock();

  // 更新协方差
  Eigen::MatrixXd I = Eigen::Matrix3d::Identity();
  sigma_ = (I - K * H) * sigma_;

  // 保存上一时刻真实的控制输入
  mutex_.lock();
  double d_theta_i = state_(2) - state_(5);
  double theta_i = state_(5);
  double dx = state_(0) - state_(3);
  double dy = state_(1) - state_(4);
  mutex_.unlock();
  double cos_theta = cos(theta_i + 0.5 * d_theta_i);
  double sin_theta = sin(theta_i + 0.5 * d_theta_i);
  if (abs(cos_theta) > 0.0001) {
    last_u_(0) = dx / cos_theta;
    last_u_(1) = d_theta_i;
  } else {
    last_u_(0) = dy / sin_theta;
    last_u_(1) = d_theta_i;
  }

}

void RobotPoseEKF::GetFusionPose(float& poseX, float& poseY, float& posePhi) {
  mutex_.lock();
  poseX = (float)state_(0);
  poseY = (float)state_(1);
  posePhi = (float)state_(2);
  mutex_.unlock();
}

void RobotPoseEKF::NormAngle(double& angle) {
  if( angle >= M_PI)
    angle -= 2*M_PI;
  if( angle < -M_PI)
    angle += 2*M_PI;
}
