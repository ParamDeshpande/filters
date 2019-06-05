// Include Files
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv/highgui.h>
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "kalman_Filter.h"
#include "main.h"
#include <fstream>
#include <iostream>


using namespace std;

std::ifstream infile("values.txt");
std::ofstream outfile("newValues.txt");

int m = 20000;

static void main_kalmanF(){
Filter F;
cv::Mat xCords, yCords, zCords, xAng, yAng, zAng;
cv::Mat xCords_kal, yCords_kal, zCords_kal, xAng_kal, yAng_kal, zAng_kal;
  xCords = cv::Mat::zeros(m, 1, CV_32F);
  yCords = cv::Mat::zeros(m, 1, CV_32F);
  zCords = cv::Mat::zeros(m, 1, CV_32F);
  xAng = cv::Mat::zeros(m, 1, CV_32F);
  yAng = cv::Mat::zeros(m, 1, CV_32F);
  zAng = cv::Mat::zeros(m, 1, CV_32F);
  xCords_kal = cv::Mat::zeros(m, 1, CV_32F);
  yCords_kal = cv::Mat::zeros(m, 1, CV_32F);
  zCords_kal = cv::Mat::zeros(m, 1, CV_32F);
  xAng_kal = cv::Mat::zeros(m, 1, CV_32F);
  yAng_kal = cv::Mat::zeros(m, 1, CV_32F);
  zAng_kal = cv::Mat::zeros(m, 1, CV_32F);

  //Load the values in the vectors 
  for(int i= 0; i<20000; i++){
  infile>> 
  xCords.at<float>(i,1) >> yCords.at<float>(i,1) >> zCords.at<float>(i,1) >> 
  xAng.at<float>(i,1) >> yAng.at<float>(i,1) >> zAng.at<float>(i,1) ;
  }
/*/Process and store those values
  cv::Mat XHat_Previous, XHat_Posterior;
  cv::Mat estVar_Previous,estVar_Priori, estVar_Posterior;
   XHat_Previous = (cv::Mat_<float>(3,1) <<
		0.000,
    0.000,
    0.000);
    XHat_Posterior = (cv::Mat_<float>(3,1) <<
		0.0000,
    0.0000,
    0.0000);

estVar_Previous = (cv::Mat_<float>(3,3) <<
		 0, 0, 0 ,
      0, 0, 0 ,
      0, 0, 0);

estVar_Posterior = (cv::Mat_<float>(3,3) <<
		 0, 0, 0 ,
      0, 0, 0 ,
      0, 0, 0);

for(int i=0; i<20000; i++){
  F.kalman(i, xCords.at<float>(i,1), XHat_Previous,
  estVar_Previous, XHat_Posterior, estVar_Posterior);
  XHat_Previous = XHat_Posterior ;
  estVar_Previous = estVar_Posterior;
  xCords_kal.at<float>(i,1) = XHat_Posterior.at<float>(i,1);
}
*/
cv::Mat XHat_Previous, XHat_Posterior;
  cv::Mat estVar_Previous,estVar_Priori, estVar_Posterior;
   XHat_Previous = (cv::Mat_<float>(3,1) <<
		0.000,
    0.000,
    0.000);
    XHat_Posterior = (cv::Mat_<float>(3,1) <<
		0.0000,
    0.0000,
    0.0000);

 estVar_Previous =  (cv::Mat_<float>(3,3) <<
		 0, 0, 0 ,
      0, 0, 0 ,
      0, 0, 0);

 estVar_Posterior = (cv::Mat_<float>(3,3) <<
		 0, 0, 0 ,
      0, 0, 0 ,
      0, 0, 0);


//trying for single value 
for(int i=0 ; i<12000; i++){
F.kalman(1,xCords.at<float>(i,1),XHat_Previous,estVar_Previous,XHat_Posterior,estVar_Posterior);
xCords_kal.at<float>(i,1) = XHat_Posterior.at<float>(i,1);
XHat_Previous = XHat_Posterior;
estVar_Previous = estVar_Posterior;
}

//Write back into the new file
  for(int i=0; i<20000; i++){
 outfile << xCords_kal.at<float>(i,1) <<"\t"<< yCords_kal.at<float>(i,1) <<"\t"<<zCords_kal.at<float>(i,1) << endl;
  }
}

int main(int, const char * const []){
  // Initialize the application.
  // You do not need to do this more than one time.
 
  // Invoke the entry-point functions.
  // You can call entry-point functions multiple times.
  main_kalmanF();

 

  // Terminate the application.
  // You do not need to do this more than one time.

  return 0;
}

//
// File trailer for main.cpp
//
// [EOF]
//