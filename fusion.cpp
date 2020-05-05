/*
 * @Description: optical flow, encoder融合定位
 * @Author: Ji Jiawen
 * @Date: 2020-04-13
 */

#include "robot_pose_ekf.h"
#include "median_filter.h"
#include <iostream>

int main(int argc, char* argv[]) {
  static int size = 20;
  int in[size], out[size];

  for (size_t i = 0; i < size; i++)
  {
    in[i] = 13;
  }
  in[1] = 0;
  in[5] = 0;
  in[6] = 0;

  in[9] = 0;
  in[10] = 0;
  in[12] = 0;


  MedianFilter median_filter;
  bool is_ok = median_filter.Filter(in, out, size, 10, (int*)0);

  if (is_ok) {
    for (size_t i = 0; i < size; i++) {
      std::cout << "in: i = " << i << " " << in[i] << std::endl;
    }

    for (size_t i = 0; i < size; i++) {
      std::cout << "out: i = " << i << " " << out[i] << std::endl;
    }
  } else {

  }



  RobotPoseEKF ekf(0.0, 0.0, 0.0, 0.0, 0.0, 
                    0.0, 0.0, 0.0, 0.0);
  
  return 0;
}