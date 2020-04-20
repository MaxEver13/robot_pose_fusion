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
  state_ = Eigen::Vector3d(0, 0, 0);
  sigma_.setZero();
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
  double delta_s_l = kl_ * delta_enc_l;
  double delta_s_r = kr_ * delta_enc_r;

  // 计算位姿变化量
  // double delta_theta = (delta_s_r - delta_s_l) / b_; 
  double delta_theta = (theta - last_theta_) / 1000.0; // 弧度
  double delta_s = 0.5 * (delta_s_r + delta_s_l) / 1000.0; // 米

  double theta_tmp = state_(2) + 0.5 * delta_theta;
  double cos_theta = cos(theta_tmp);
  double sin_theta = sin(theta_tmp);

  double delta_x = delta_s * cos_theta;
  double delta_y = delta_s * sin_theta;

  // 更新当前位姿的变化量
  u_(0) = delta_x;
  u_(1) = delta_y;
  u_(2) = delta_theta;

  // 更新状态
  mutex_.lock();
  state_(0) += delta_s * cos_theta;
  state_(1) += delta_s * sin_theta;
  state_(2) += delta_theta;
  NormAngle(state_(2));
  mutex_.unlock();  

  // 运动模型关于上一时刻位姿的雅各比
  Eigen::Matrix3d F;
  F << 1.0, 0.0, -delta_s * sin_theta,
        0.0, 1.0, delta_s * cos_theta,
        0.0, 0.0, 1.0;

  // 运动模型关于上一时刻控制输入的雅各比
  Eigen::Matrix3d G; 
  G << 0.5 * cos_theta, 0.5 * cos_theta, - 0.5 * delta_s * sin_theta,
        0.5 * sin_theta, 0.5 * sin_theta, 0.5 * delta_s * cos_theta,
        0.0, 0.0, 1.0;

  // 控制输入协方差
  Eigen::Matrix3d sigma_u;
  sigma_u << k_ * k_ * delta_s_r * delta_s_r, 0.0, 0.0,
            0.0, k_ * k_ * delta_s_l * delta_s_l, 0.0,
            0.0, 0.0, (0.2*0.0174533)*(0.2*0.0174533)*dt*dt;

  // 更新协方差
  sigma_ = F * sigma_ * F.transpose() + G * sigma_u * G.transpose();

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
  double psi_tmp = state_(2) + psi_;
  mutex_.unlock();
  double sin_psi = sin(psi_tmp);
  double cos_psi = cos(psi_tmp);

  Eigen::Matrix<double, 2, 3> A;
  A << 1.0, 0.0, -l_ * sin_psi, 
        0.0, 1.0, l_ * cos_psi;

  // 光流传感器到世界坐标系的旋转
  mutex_.lock();
  double phi_tmp = state_(2) + phi_;
  mutex_.unlock();
  double sin_phi = sin(phi_tmp);
  double cos_phi = cos(phi_tmp);
  Eigen::Matrix2d R;
  R << cos_phi, -sin_phi,
        sin_phi, cos_phi;

  // 光流传感器的测量协方差
  Eigen::Matrix2d Q;
  Q << ksx_ * ksx_ * sx * sx, 0.0,
        0.0, ksy_ * ksy_ * sy * sy;

  // 光流传感器观测的估计
  Eigen::Vector2d z_hat = R.transpose() * A * u_;

  // 测量残差
  Eigen::Vector2d residual = Eigen::Vector2d(sx, sy) - z_hat;

  // 残差关于位姿变化量的雅各比
  Eigen::MatrixXd H = - R.transpose() * A;

  // ESKF
  const Eigen::MatrixXd& P = sigma_;
  const Eigen::MatrixXd K = P * H.transpose() * (H * P * H.transpose() + Q).inverse();
  const Eigen::VectorXd delta_x = K * residual;

  // 更新状态
  mutex_.lock();
  state_(0) += delta_x(0);
  state_(1) += delta_x(1);
  state_(2) += delta_x(2); 
  NormAngle(state_(2));
  mutex_.unlock();

  // 更新协方差
  // Eigen::MatrixXd I = Eigen::Matrix<double, ３, ３>::Identity()
  // sigma_ = (I - K * H) * sigma_;
  const Eigen::MatrixXd I_KH = Eigen::Matrix<double, 3, 3>::Identity() - K * H;
  sigma_ = I_KH * P * I_KH.transpose() + K * Q * K.transpose();
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
