// Include Files
#include "constants.h"
#include "kalman_Filter.h"

float Filter::kalman(int i,float measurement,cv::Mat &XHat_Previous,cv::Mat &estVar_Previous){

//*****equation matrices**********/
// state transition matrix
A = (cv::Mat_<float>(3,3) <<
		1, alpha*delT, (delT*delT)/2,
		0, 1, beta*delT,
		0, 0, gamma);
//Control Matrix 
B = ((cv::Mat_<float>(3,1))<<
                    0,        
                    0,        
  measurement - XHat_Previous.at<float>(0,0)  );
// Process noise
W = (cv::Mat_<float>(3,1) <<
		0.0001,
    0.0002,
    0.0001);

// process noise covariance
Q = (cv::Mat_<float>(3,3) <<
    posVarSq, 0, 0 ,
    0, velVarSq, 0 ,
    0, 0, accVarSq);

// measurement noise
V = (cv::Mat_<float>(3,3) <<
    0.0005,
    0.0001,
    0.0001);
//Scale: OP = scale*xHat
scale  = (cv::Mat_<float>(3,3) <<
    1,
    1,
    1);


// Initialization
if (i == 1.0) {
    XHat_Previous = ((cv::Mat_<float>(3,1))<<
    0,
    0,
    0);


    float estVar_Pos = 100;
    float estCovar_Pos_Vel = 91;
    float estVar_Vel = 100;
    float estCovar_Vel_Acc = 71;
    float estVar_Acc = 100;
    float estCovar_Pos_Acc = 51;
    
    estVar_Previous = (cv::Mat_<float>(4, 4) <<
		estVar_Pos, estCovar_Pos_Vel, estCovar_Pos_Acc,
	    estCovar_Pos_Vel, estVar_Vel, estCovar_Vel_Acc,
		estCovar_Pos_Acc, estCovar_Vel_Acc, estVar_Acc);

  }
 


// **********state extrapolation********
XHat_Priori = A*XHat_Previous + B + W ;

// **********covariance extrapolation******
estVar_Priori = A*estVar_Previous*A.t() + Q;

//***********Kalman Gain**********
kalman_Gain = (estVar_Priori*scale.t())*((scale*estVar_Priori*scale.t() + R).inv());

//********State update equation*******
XHat_Posterior = XHat_Priori + kalman_Gain*(measurement - scale*XHat_Priori);

//********Covariance update equation********
estVar_Posterior = ((cv::Mat::eye(3,3,CV_32F)) - kalman_Gain*scale)*estVar_Priori ;

XHat_Posterior.copyTo(XHat_Previous);
estVar_Posterior.copyTo(estVar_Previous);

return XHat_Posterior.at<float>(0,0); //return the position

}

