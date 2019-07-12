xAng_Velocity = IMU_data(:,2);
ax = IMU_data(:,5);
ay = IMU_data(:,6);
az = IMU_data(:,7);

xAng = zeros(35000,1);

xAng_Gyro = zeros(35000,1);
xAng_Gyro_Clean = zeros(35000,1);

xAng_Acc = zeros(35000,1);
xAng_Acc_Clean = zeros(35000,1);


M_Freq_Hz = 200;
delT = 1/M_Freq_Hz;

%Freq below this will be blocked (for gyroscope)
HP_cutoff_Freq_Hz = 0.001;

%Freq above this will be blocked (for accelerometer)
LP_cutoff_Freq_Hz = 150;

% Per cent contrib due to gyroscope and accelorometer
alpha_CF = 0.95;

HPF_tau = 1/(2*3.1415*HP_cutoff_Freq_Hz);
LPF_tau = 1/(2*3.1415*LP_cutoff_Freq_Hz);

alpha_HP = (HPF_tau/(HPF_tau+delT));
alpha_LP = (LPF_tau/(LPF_tau+delT));

for i = 2:35000

    % Gyro 
    xAng_Gyro(i,1) = xAng_Gyro(i-1,1) + xAng_Velocity(i,1)*delT;
    xAng_Gyro_Clean(i,1) = hp_Filter(xAng_Gyro_Clean(i-1,1),xAng_Gyro(i,1),xAng_Gyro(i-1,1),alpha_HP);
    
    %Acc
    xAng_Acc(i) = atan(ax(i)/( ((ay(i)^2)+(az(i)^2))^0.5) );
    xAng_Acc_Clean(i) = lp_Filter(xAng_Acc_Clean(i-1), xAng_Acc(i,1), alpha_LP );

    %Complimenatary
    xAng(i) = (alpha_CF)*xAng_Gyro_Clean(i) + (1-alpha_CF)*xAng_Acc_Clean(i);
    
end
