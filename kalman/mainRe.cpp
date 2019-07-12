#include "main.h"
#include "filter.h"
#include "constants.h"

static void main_Kalman(){
    std::ifstream infile("values.txt");
    std::ofstream outfile("newValues.txt");
    int m = 20000;
    Filter filter;
    cv::Mat xCords, yCords, zCords, xAng, yAng, zAng;
    cv::Mat xCords_kal, yCords_kal, zCords_kal, xAng_kal, yAng_kal, zAng_kal;
    //Init all cords,angles as zeros ...
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
    for(int i= 0; i<19000; i++){
        infile>> 
        xCords.at<float>(i,0) >> yCords.at<float>(i,0) >> zCords.at<float>(i,0) >> 
        xAng.at<float>(i,0) >> yAng.at<float>(i,0) >> zAng.at<float>(i,0) ;
     }
    //Defining matrices IP and OP
    cv::Mat XHat_Previous, estVar_Previous;

        XHat_Previous = (cv::Mat_<float>(3,1) <<
	    0.000,
        0.000,
        0.000);


        estVar_Previous =  (cv::Mat_<float>(3,3) <<
 	    0, 0, 0 ,
        0, 0, 0 ,
        0, 0, 0);

    
    //Passing through filter and writing out to the new file
    for(int i=0 ; i<18000; i++){
        xCords_kal.at<float>(i,0) = filter.kalman(xCords.at<float>(i,0),XHat_Previous,estVar_Previous);
        outfile << xCords_kal.at<float>(i,0) << std::endl;
        }

}

int main(int, const char * const []){
  
  main_Kalman();
  
  return 0;
}
