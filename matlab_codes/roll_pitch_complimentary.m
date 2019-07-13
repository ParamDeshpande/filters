dt = 1/200;
t = 25000;
truedata = TRUE_data;
imudata = IMU_data;
trueang = zeros(t,2);
%quad to angles
for i = 1:t
    quat = [truedata(i,6) truedata(i,7) truedata(i,8) truedata(i,5)];
    y = quat2eul(quat,'XYZ');
    trueang(i,1) = y(1) - 2.4277;
    trueang(i,1) =remap(trueang(i,1));
    trueang(i,2) = y(2);
    trueang(i,2) =remap(trueang(i,2));
end

imu = [imudata(:,2) imudata(:,3) imudata(:,4) imudata(:,5) imudata(:,6) imudata(:,7)];
Ax = imu(:,4);
Ay = imu(:,5);
Az = imu(:,6);
Gx = imu(:,1);
Gy = imu(:,2);
Gz = imu(:,3);

% Convert gyroscope measurements from degrees to radians
Gx_rad = Gx; %* pi / 180.0;
Gy_rad = Gy; %* pi / 180.0;
Gz_rad = Gz; %* pi / 180.0;

%phi_hat_acc = zeros(t,2);
%theta_hat_acc = zeros(t,2); 
%A = zeros(t,2);
for i = 1:t
    phi_hat_acc   = atan2(Ay(i), sqrt(Ax(i)^ 2 + Az(i)^ 2));
    theta_hat_acc = atan2(-Ax(i), sqrt(Ay(i)^ 2 + Az(i)^ 2));
    A(i,1) = [phi_hat_acc];
    A(i,2) = [theta_hat_acc];
end

% 2) Gyroscope only
phi_hat_gyr   = zeros(1, length(t));
theta_hat_gyr = zeros(1, length(t));

for i = 2:t
   p = Gx_rad(i);
   q = Gy_rad(i);
   r = Gz_rad(i);
   
   phi_hat   = phi_hat_gyr(i - 1);
   
   
   theta_hat = theta_hat_gyr(i - 1);
   
   phi_hat_gyr(i)   = phi_hat   + dt * p;
   %phi_hat_gyr(i)   = phi_hat   + dt * (p + sin(phi_hat) * tan(theta_hat) * q + cos(phi_hat) * tan(theta_hat) * r);
   phi_hat_gyr(i) = remap(phi_hat_gyr(i));
   
   theta_hat_gyr(i) = theta_hat + dt * q;
   %theta_hat_gyr(i) = theta_hat + dt * (cos(phi_hat) * q - sin(phi_hat) * r);
   theta_hat_gyr(i) = remap(theta_hat_gyr(i));
   
    G(i,1) = [phi_hat_gyr(i) ];
    G(i,2) = [theta_hat_gyr(i)];
end
    

% 3) Complimentary Filter
alpha = 0.6;
C = zeros(t,2);
for i=1:t
   C(i,1) = alpha*G(i,1) + (1-alpha)*A(i,1);
   C(i,2) = alpha*G(i,2) + (1-alpha)*A(i,2);
end
