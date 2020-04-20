/*
 * @Description: 中值滤波
 * @Author: Ji Jiawen
 * @Date: 2020-04-20
 */

#include "median_filter.h"

template<typename Type>
MedianFilter::MedianFilter(Type* input, Type* output, int size, int kernel_size, Type* buf) {
  bool newBuf = false;
  if(buf == 0) {
    newBuf = true;
    buf = new Type[kernel_size];
  }
  int half = kernel_size / 2;
  int end = size - half;

  // 数组左端
  for(int pos = 0; pos < half; pos ++) {
    int part = 2 * pos + 1; // part即window size大小
    copy(input, buf, 0, part);
    QuickSort(buf, part);
    output[pos] = buf[part / 2];
  }
  // 数组中间
  int left_bound = 0;
  for(int pos = half; pos < end; pos ++) {
    copy(input, buf, left_bound, kernel_size);
    left_bound++;
    QuickSort(buf, kernel_size);
    output[pos] = buf[half];
  }
  // 数组右端
  left_bound++;
  for(int pos = end; pos < size; pos ++) {
    int part = 2 * (size - pos) - 1;
    copy(input, buf, left_bound, part);
    left_bound += 2;
    QuickSort(buf, part);
    output[pos] = buf[part / 2];
  }
  if(newBuf) {
    delete[] buf;
  }
}

template<typename Type>
void MedianFilter::QuickSort(Type* arr, int n) {
  QSort(arr, 0, n - 1);
}

template<typename Type>
void MedianFilter::QSort(Type* arr, int p, int r) {
  if (p <= r) {
    int	q = Partition(arr, p, r);
    QSort(arr, p, q - 1);
    QSort(arr, q + 1, r);
  }	
}

template<typename Type>
int MedianFilter::Partition(Type* arr, int low, int high) {
  int key = arr[high]; // 这里可以随机选择一个数组中的元素，选择最后一个
	int i = low-1; // 小于等于区间的下标
	for (int j = low; j < high; j++) {
		if (arr[j] <= key) // 如果当前值小于等于key,小于等于区间下标加１，将当前值放到小于等于区间(原址)
		{
			i++; 
			int temp = arr[i]; // 跟小于等于区间的后一个数交换
			arr[i] = arr[j];
			arr[j] = temp;
		}
	}
	int temp = arr[i + 1]; // 将选中的key放到小于等于区间后面(跟小于等于区间后一个数交换)，这样原数组被key分为两段
	arr[i + 1] = arr[high]; 
	arr[high] = temp;
	return i + 1; //　返回选中key在划分后数组的下标
}
