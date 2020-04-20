/*
 * @Description: optical flow, encoder融合定位
 * @Author: Ji Jiawen
 * @Date: 2020-04-13
 */

#include "robot_pose_ekf.h"
#include "median_filter.h"

int main(int argc, char* argv[]) {
  int in[100], out[100];

  MedianFilter median_filter;
  median_filter.Filter(in, out, 100, 5, (int*)0);

  RobotPoseEKF ekf(0.0, 0.0, 0.0, 0.0, 0.0, 
                    0.0, 0.0, 0.0, 0.0);
  
  return 0;
}