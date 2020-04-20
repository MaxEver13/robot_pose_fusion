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
  template<typename Type>
  MedianFilter(Type* input, Type* output, int size, int kernel_size, Type* buf);
  ~MedianFilter() {};

private:
  template<typename Type>
  void QuickSort(Type* arr, int n);
  template<typename Type>
  void QSort(Type* arr, int p, int r);
  template<typename Type>
  int Partition(Type* arr, int low, int high);

  template<typename Type>
  static inline void copy(Type* input,  Type* buf, int left_bound, int kernel_size) {
    for(int i = 0; i < kernel_size; i ++) {
        buf[i] = input[left_bound + i];
    }
  }    


};


#endif