/*
 * @Description: optical flow, encoder融合定位
 * @Author: Ji Jiawen
 * @Date: 2020-04-13
 */

#ifndef ROBOT_POSE_EKF_H
#define ROBOT_POSE_EKF_H

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <mutex>

class RobotPoseEKF
{
public: 
  RobotPoseEKF(double b, double kl, double kr, double k, 
                    double l, double psi, double phi, double ksx, double ksy);
  ~RobotPoseEKF() {};

  // 获取编码器,imu数据预测状态
  void AddEncoderData(unsigned int time, const int32_t& enc_l, const int32_t& enc_r, const int32_t& phi);
  // 获取光流传感器数据更新状态
  void OpticalFlowUpdate(const double& sx, const double& sy);
  // 对外提供位姿
  void GetFusionPose(float& poseX, float& poseY, float& posePhi);

private:
  void NormAngle(double& angle);


private:
  double b_;      // 左右轮间距
  double kl_;     // 左轮编码器误差常数
  double kr_;     // 右轮编码器误差常数
  double k_;      // 控制输入的协方差参数

  double l_;  // 光流坐标系与机器人坐标系的距离
  double psi_; // 光流坐标系原点与机器人坐标系ｘ轴夹角
  double phi_; // 光流传感器坐标系x轴与机器人坐标系ｘ轴夹角
  double ksx_; // 光流测量协方差参数
  double ksy_; // 光流测量协方差参数

  bool is_inited_;  // 系统初始化标志位

  Eigen::Matrix<double, 6, 1> state_;  //　当前时刻EKF状态
  Eigen::Matrix<double, 6, 6> sigma_;  // 当前时刻状态的协方差

  unsigned int last_time_; // 数据时间戳单位毫秒
  int32_t last_enc_l_, last_enc_r_; // 上一次编码器的读数
  int32_t last_theta_; // 上一次陀螺仪z轴积分角度

  Eigen::Vector2d last_u_; // 上一时刻机器人控制输入

  std::mutex mutex_;
};

#endif