/*
 * @Description: 中值滤波,用于处理光流测量数据野值
 * @Author: Ji Jiawen
 * @Date: 2020-04-20
 */

#ifndef ROBOT_POSE_FUSION_MEDIAN_FILTER_H
#define ROBOT_POSE_FUSION_MEDIAN_FILTER_H

class MedianFilter
{
public:
  
  MedianFilter() {};
  ~MedianFilter() {};

  void Filter(int* input, int* output, int size, int kernel_size, int* buf);

private:

  void QuickSort(int* arr, int n);

  void QSort(int* arr, int p, int r);

  int Partition(int* arr, int low, int high);

  static inline void copy(int* input,  int* buf, int left_bound, int kernel_size) {
    for(int i = 0; i < kernel_size; i ++) {
        buf[i] = input[left_bound + i];
    }
  }    


};


#endif