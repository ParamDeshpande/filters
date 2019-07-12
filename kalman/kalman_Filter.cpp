// Include Files
#include "constants.h"
#include "filter.h"

float Filter::kalman(float measurement,cv::Mat &XHat_Previous,cv::Mat &estVar_Previous){

  ///*****equation matrices**********/
  // state transition matrix
  A = (cv::Mat_<float>(3,3) <<
		1, alpha*delT, (delT*delT)/2,
		0, 1, beta*delT,
		0, 0, 1);
  //Control Matrix 
  B = ((cv::Mat_<float>(3,1))<<
                    0,        
                    0,        
  gamma*(measurement - XHat_Previous.at<float>(0,0))/(delT*delT)  );
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
  scale  = (cv::Mat_<float>(1,3) <<
    1,
    1,
    1);

  //*/// **********state extrapolation********
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
/*End Of file *
A = [
  
    1    0    0    0    0    0    delT     0     0     0     0     0  ;
    0    1    0    0    0    0    0     delT     0     0     0     0  ;
    0    0    1    0    0    0    0     0     delT     0     0     0  ;
    
    0    0    0    1    0    0    0     0     0     delT     0     0;
    0    0    0    0    1    0    0     0     0     0     delT     0;
    0    0    0    0    0    1    0     0     0     0     0     delT;
    
    0    0    0    0    0    0    1     0     0     0     0     0 ;
    0    0    0    0    0    0    0     1     0     0     0     0 ;
    0    0    0    0    0    0    0     0     1     0     0     0 ;
    
    0    0    0    0    0    0    0     0     0     1     0     0 ;
    0    0    0    0    0    0    0     0     0     0     1     0 ;
    0    0    0    0    0    0    0     0     0     0     0     1 ;

];

B = [
      
       0        0        0        0.5*(delT)*(delT)        0        0 ;  
       0        0        0        0        0.5*(delT)*(delT)        0 ;  
       0        0        0        0        0        0.5*(delT)*(delT) ;  
       
       delT        0        0        0        0        0      ;
       0        delT        0        0        0        0      ;
       0        0        delT        0        0        0  ;

       0        0        0        delT        0        0   ;
       0        0        0        0        delT        0   ;
       0        0        0        0        0        delT;

       1        0        0        0        0        0   ;
       0        1        0        0        0        0   ;
       0        0        1        0        0        0   ;
       
];

Q = [
          var_gx   0   0   0   0   0  ;  
          0   var_gy   0   0   0   0  ;
          0   0   var_gz   0   0   0  ;

          0   0   0   var_ax   0   0 ;
          0   0   0   0   var_ay   0 ;
          0   0   0   0   0   var_az ;

];

R = [

      var_px       0       0       0       0       0;
      0       var_py       0       0       0       0;
      0       0       var_pz       0       0       0;
      0       0       0       var_vx       0       0;
      0       0       0       0       var_vy       0;
      0       0       0       0       0       var_vz;

];


C = [




                      0     0 0 0 0 0 
                      0     0 0 0 0 0 
                      0     0 0 0 0 0 
                      0     0 0 0 0 0 
                      0     0 0 0 0 0 
                      0     0 0 0 0 0 
                      0     0 0 0 0 0 
                      0     0 0 0 0 0 
                      0     0 0 0 0 0 

];

