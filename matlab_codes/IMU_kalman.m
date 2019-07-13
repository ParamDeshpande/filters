m = 36000;

xpos = zeros(m,1);
xpos(1,1) = 4.69;
xvel = zeros(m,1);
ax = zeros(m,1);

ypos = zeros(m,1);
yvel = zeros(m,1);
ay = zeros(m,1);

zpos = zeros(m,1);
zvel = zeros(m,1);
az = zeros(m,1);


roll = zeros(m,1);
roll_w = zeros(m,1);
roll_acc = zeros(m,1);

pitch = zeros(m,1);
pitch_w = zeros(m,1);
pitch_acc = zeros(m,1);

yaw = zeros(m,1);
yaw_w = zeros(m,1);
yaw_acc = zeros(m,1);

newSTATE = zeros(m,18);
imu = [IMU_data(:,2) IMU_data(:,3) IMU_data(:,4) IMU_data(:,5) IMU_data(:,6) IMU_data(:,7)];
imu_inst = zeros(6,1);
gps = [gps_data(:,2) gps_data(:,3) gps_data(:,4)];
gps_inst = zeros(3,1);

fps_Hz = 200;
i_prev = 0;
j= 1;

P = eye(4);
%mag_x = magReadX*cos(pitch) + magReadY*sin(roll)*sin(pitch) + magReadZ*cos(roll)*sin(pitch)
%mag_y = magReadY * cos(roll) - magReadZ * sin(roll)
%yaw = 180 * atan2(-mag_y,mag_x)/M_PI;

for i = 2:m
    
pitch = 180 * atan2(ax, sqrt(ay*ay + az*az))/PI;
roll = 180 * atan2(ay, sqrt(ax*ax + az*az))/PI;
    
    
state_New = [ xpos(i); ypos(i); zpos(i); roll(i); pitch(i); yaw(i); 
          xvel(i); yvel(i); zvel(i); roll_w(i); pitch_w(i); yaw_w(i);
          ax(i); ay(i); az(i); roll_acc(i); pitch_acc(i); yaw_acc(i)];
    
    imu_inst = [imu(i,1);
                imu(i,2);
                imu(i,3);
                imu(i,4);
                imu(i,5);
                imu(i,6);
                
    ];

    if((i - i_prev) > 11)
        j = j+1;
        i_prev = i;
    end
        gps_inst = [   gps(j,1);
                       gps(j,2);
                       gps(j,3);
                       ];
    
    [state_New, cov_New] = kalman6d(imu_inst, gps_inst, state_New, cov_New,fps_Hz);
    for p = 1:18
        newSTATE(i,p) = state_New(p);
    end
end
positionX = newSTATE(:,4);
positionTRUE = TRUE_data(1:m,6);