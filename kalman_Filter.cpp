// Include Files
#include "kalman_Filter.h"
#include <opencv2/highgui.hpp>
#include <iostream>

//Member functions 
void Filter::kalman(int i,float measurement,cv::Mat XHat_Previous,
cv::Mat estVar_Previous,cv::Mat XHat_Posterior,
cv::Mat estVar_Posterior){

//by how much MAX percent can they change each iteration 
float alpha = 0.9;
float beta = 0.095 ;
float gamma = 0.01;
float delT =  1;  

 // ******equation matrices**********
 // state transition matrix
A = (cv::Mat_<float>(3,3) <<
		1, alpha*delT, (delT*delT)/2,
		0, 1, beta*delT,
		0, 0, gamma/(delT*delT));

// Process noise
  W = (cv::Mat_<float>(3,1) <<
		0.0001,
    0.0002,
    0.0001);

      // process noise covariance
 float posVarSq = 1e5;
 float velVarSq = 1e5; 
 float accVarSq = 1e5;

Q = (cv::Mat_<float>(3,3) <<
		 posVarSq, 0, 0 ,
      0, velVarSq, 0 ,
      0, 0, accVarSq);

// measurement noise
  V = (cv::Mat_<float>(3,1) <<
    0.0005,
    0.0001,
    0.0001);

// Measurement noise co-variance
  float mPosVarSq = 20000;
  float R = mPosVarSq;
  
  scale  = (cv::Mat_<float>(1,3) <<
    1,
    1,
    1);

//Noise addition to measurements 
  measurement += 0.0005;

  // Initialization
  if (i == 1.0) {
    XHat_Previous = (cv::Mat_<float>(3,1)<<
    0,
    0,
    0);

    float estVar_Pos = 100;
    float estCovar_Pos_Vel = 91;
    float estVar_Vel = 100;
    float estCovar_Vel_Acc = 71;
    float estVar_Acc = 100;
    float estCovar_Pos_Acc = 51;
    
    estVar_Previous = (cv::Mat_<float>(3, 3) <<
		estVar_Pos, estCovar_Pos_Vel, estCovar_Pos_Acc,
	  estCovar_Pos_Vel, estVar_Vel, estCovar_Vel_Acc,
		estCovar_Pos_Acc, estCovar_Vel_Acc, estVar_Acc);

    XHat_Priori = cv::Mat::zeros(3,1,CV_32F);
    estVar_Priori = cv::Mat::eye(3,3,CV_32F);
  }

acc_Update = (cv::Mat_<float>(3,1)<<
0,
0,
measurement - XHat_Previous.at<float>(1,1));

// **********state extrapolation********
XHat_Priori = A*XHat_Previous + acc_Update + W ;

// **********covariance extrapolation******
estVar_Priori = A*estVar_Previous*(A.t()) + Q;

//***********Kalman Gain**********
kalman_Gain = (estVar_Priori*scale.t())*((scale*estVar_Priori*scale.t() + R ).inv());

//********State update equation*******
XHat_Posterior = XHat_Priori + kalman_Gain*(measurement - scale*XHat_Priori);

//********Covariance update equation********
estVar_Posterior = ((cv::Mat::eye(3, 3, CV_32F) - kalman_Gain*scale)*estVar_Priori);
}
