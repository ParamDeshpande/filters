dt = 1/200;
t = 25000;

truedata = TRUE_data;
imudata = IMU_data;

trueang = zeros(t,2);

for i = 1:t
    quat = [truedata(i,6) truedata(i,7) truedata(i,8) truedata(i,5)];
    y = quat2eul(quat,'XYZ');
    
    trueang(i,1) = y(1);
    trueang(i,1) =remap(trueang(i,1));
    
    trueang(i,2) = y(2);
    trueang(i,2) = remap(trueang(i,2));

end
%import IMU_data file as "imudata"
imu = [imudata(:,2) imudata(:,3) imudata(:,4) imudata(:,5) imudata(:,6) imudata(:,7)];
Ax = imu(:,4);
Ay = imu(:,5);
Az = imu(:,6);
Gx = imu(:,1);
Gy = imu(:,2);
Gz = imu(:,3);

% Convert gyroscope measurements from degrees to radians or according to your measurements
Gx_rad = Gx; %* pi / 180.0;
Gy_rad = Gy; %* pi / 180.0;
Gz_rad = Gz; %* pi / 180.0;

% For this kalman filter it is assumed that gyro gives aka predicts values and 
% accelerometers values are considered as measurement benchmarks
% Kalman gain is % belief between gyro and acc readings
% May add GPS values as measurents and imu as prediction (not recommended)
%State vector is phi , phi bias , theta , theta bias

state = [0; 2.47; 0; 2.47];
state_Matrix = zeros(t,4);
P = eye(4);

state_comp = zeros(1,2);
state_matrix_comp = zeros(t,2);

phi_hat = zeros(t,1);
bias_phi = zeros(t,1);

theta_hat = zeros(t,1);
bias_theta = zeros(t,1);


for i = 2:t
    p = Gx_rad(i);
    q = Gy_rad(i);
    r = Gz_rad(i);
    
    state = state_Matrix(i-1,:);
    phi_hat   = state(1);
    bias_phi =  state(2);
    theta_hat = state(3);
    bias_theta = state(4);
    
    phi_hat_acc(i)   = atan2(Ay(i), sqrt(Ax(i)^ 2 + Az(i)^ 2));
    phi_hat_acc(i) = remap(phi_hat_acc(i));
    
    theta_hat_acc(i) = atan2(-Ax(i), sqrt(Ay(i)^ 2 + Az(i)^ 2));
    theta_hat_acc(i) = remap(theta_hat_acc(i));
    
    phi_dot   = p + sin(phi_hat) * tan(theta_hat) * q + cos(phi_hat) * tan(theta_hat) * r;
    theta_dot = cos(phi_hat) * q - sin(phi_hat) * r;
    
    phi_hat_gyr(i)   = phi_hat   + dt * p;
    %phi_hat_gyr   = phi_hat   + dt * (p + sin(phi_hat) * tan(theta_hat) * q + cos(phi_hat) * tan(theta_hat) * r);
    theta_hat_gyr(i) = theta_hat + dt * q;
    %theta_hat_gyr = theta_hat + dt * (cos(phi_hat) * q - sin(phi_hat) * r);
       
    state = [phi_hat; bias_phi; theta_hat; bias_theta];
    U =  [phi_dot; theta_dot]; 
    measure =  [phi_hat_acc(i); theta_hat_acc(i)];
    [state,P] = kalman6d(U,measure,state,P,200);
    
    for j = 1:4
        state_Matrix(i,j) = state(j,1);
    end
end
for i=1:t
    state_Matrix(i,1) = state_Matrix(i,1);
    state_Matrix(i,3) = state_Matrix(i,3);
end
% May convert all estimates to degree accordingly ddepending on your values
%********************END OF FILE********************
