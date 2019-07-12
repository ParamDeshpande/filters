%P = eye(4);
%state_estimate = [0 0 0 0]';

function [ XHat_Posterior,estVar_Posterior ] = kalmanF( U,Z ,XHat_Previous , estVar_Previous ,freq_Hz)

dt = 1/freq_Hz;
%******equation matrices**********

A = [1 -dt 0 0; 0 1 0 0; 0 0 1 -dt; 0 0 0 1];
B = [dt 0 0 0; 0 0 dt 0]';
C = [1 0 0 0; 0 0 1 0];
Q = eye(4) * 0.01;
R = eye(2) * 10;


%**********state extrapolation********
XHat_Priori = A*XHat_Previous + B*U;
%XHat_Priori = A*XHat_Previous + acc_Update ;

%**********covariance extrapolation******
estVar_Priori = A*estVar_Previous*A' + Q; 

%***********Kalman Gain**********
gain = (estVar_Priori*C')*((C*estVar_Priori*C' + R )^-1);

%********State update equation*******
XHat_Posterior = XHat_Priori + gain*(Z - C*XHat_Priori);

%********Covariance update equation********
estVar_Posterior = (eye(size(XHat_Posterior)) - gain*C)*estVar_Priori ;


end
