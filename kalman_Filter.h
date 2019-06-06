//
// File: kalmanF.h
//
// MATLAB Coder version            : 4.1
// C/C++ source code generated on  : 03-Jun-2019 10:57:41
//
#ifndef KALMAN_FILTER_H
#define KALMAN_FILTER_H

// Include Files
#include "main.h"

class Filter{
public:
//Data Members 
cv::Mat A, W, Q, V;
cv::Mat scale,kalman_Gain;
cv::Mat XHat_Previous,XHat_Priori, XHat_Posterior;
cv::Mat estVar_Previous,estVar_Priori, estVar_Posterior;
cv::Mat B;
int i;
double measurement;

//Member functions 
float kalman(int i,float measurement,cv::Mat &XHat_Previous,cv::Mat &estVar_Previous);
};

#endif

//
// File trailer for kalmanF.h
//
// [EOF]
//
