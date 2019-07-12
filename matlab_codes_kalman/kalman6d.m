function [ XHat_Updated , cov_Updated ] = kalman6d( U, Z ,XHat_Prev , cov_Prev , fps_Hz)

    dt =  1/fps_Hz;
    
    A = ([[1, -dt, 0, 0]; 
        [0, 1, 0, 0];
        [0, 0, 1, -dt];
        [0, 0, 0, 1]]);
    
    B = ([[dt, 0];
        [0, 0];
        [0, dt];
        [0, 0]]);
    
    H = ([[1, 0, 0, 0]; [0, 0, 1, 0]]);
    Q = eye(4);
    R = 1000*eye(2);
    
    %******equation matrices***********

    %**********state extrapolation********
    XHat_Priori = A*XHat_Prev + B*U;


    %**********covariance extrapolation******
    cov_Priori = A*cov_Prev*A' + Q; 


    %***********Kalman Gain**********
    gain = (cov_Priori*H')*((H*cov_Priori*H' + R )^-1);


    %********State update equation*******
    innovation = (Z - H*XHat_Priori);
    XHat_Updated = XHat_Priori + gain*innovation;


    %********Covariance update equation********
    cov_Updated = (eye(size(XHat_Priori)) - gain*H)*cov_Priori ;

    %XHat_Updated = (XHat_Prev);
    %cov_Updated = (cov_Prev);
end