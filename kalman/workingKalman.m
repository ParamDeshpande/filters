xCords = Untitled(:,2);
yCords = Untitled(:,3);
zCords = Untitled(:,4);
xAngles = Untitled(:,4);
yAngles = Untitled(:,2);
zAngles = Untitled(:,3);
[m,n] = size(Untitled);

xNewCords = zeros(size(xCords));
outlierStatusXCords = zeros(m,1);
approxValXCords = zeros(m,1);

yNewCords = zeros(size(yCords));
outlierStatusYCords = zeros(m,1);
approxValYCords = zeros(m,1);

zNewCords = zeros(size(zCords));
outlierStatusZCords = zeros(m,1);
approxValZCords = zeros(m,1);

xNewAngles = zeros(size(xAngles));
outlierStatusX = zeros(m,1);
approxValX = zeros(m,1);

yNewAngles = zeros(size(yAngles));
outlierStatusY = zeros(m,1);
approxValY = zeros(m,1);

zNewAngles = zeros(size(zAngles));
outlierStatusZ = zeros(m,1);
approxValZ = zeros(m,1);

%MAD FILTER 
for j = 100:m
   [outlierStatusXCords(j,1),approxValXCords(j,1)] = MADFilter(j,1,Untitled,100);
   if(outlierStatusX(j) == 1)% the value is an outlier make it a median value
       xNewCords(j,1) = 0;
       xNewCords(j,1) = approxValXCords(j,1);
   else
       xNewCords(j,1) = xCords(j,1);
   end
end

for j = 100:m
   [outlierStatusYCords(j,1),approxValYCords(j,1)] = MADFilter(j,2,Untitled,100);
   if(outlierStatusY(j) == 1)% the value is an outlier make it a median value
       yNewCords(j,1) = 0;
       yNewCords(j,1) = approxValYCords(j,1);
   else
       yNewCords(j,1) = yCords(j,1);
   end
end

for j = 100:m
   [outlierStatusZCords(j,1),approxValZCords(j,1)] = MADFilter(j,3,Untitled,100);
   if(outlierStatusZ(j) == 1)% the value is an outlier make it a median value
       zNewCords(j,1) = 0;
       zNewCords(j,1) = approxValZCords(j,1);
   else
       zNewCords(j,1) = zCords(j,1);
   end
end

for j = 100:m
   [outlierStatusX(j,1),approxValX(j,1)] = MADFilter(j,4,Untitled,100);
   if(outlierStatusX(j) == 1)% the value is an outlier make it a median value
       xNewAngles(j,1) = 0;
       xNewAngles(j,1) = approxValX(j,1);
   else
       xNewAngles(j,1) = xAngles(j,1);
   end
end

for j = 100:m
   [outlierStatusY(j,1),approxValY(j,1)] = MADFilter(j,5,Untitled,100);
   if(outlierStatusY(j) == 1)% the value is an outlier make it a median value
       yNewAngles(j,1) = 0;
       yNewAngles(j,1) = approxValY(j,1);
   else
       yNewAngles(j,1) = yAngles(j,1);
   end
end

for j = 100:m
   [outlierStatusZ(j,1),approxValZ(j,1)] = MADFilter(j,6,Untitled,100);
   if(outlierStatusZ(j) == 1)% the value is an outlier make it a median value
       zNewAngles(j,1) = 0;
       zNewAngles(j,1) = approxValZ(j,1);
   else
       zNewAngles(j,1) = zAngles(j,1);
   end
end


xCord_KalPos  = zeros(m,1);
xCords_KalVel = zeros(m,1);
xCords_KalAcc = zeros(m,1);

 XHat_Old = [0; 0; 0];
 estVar_Old = 0*ones(3,3);
 yCordKal  = zeros(m,1); 
 zCordKal  = zeros(m,1);
 xAngleKal = zeros(m,1);
 yAngleKal = zeros(m,1);
 zAngleKal = zeros(m,1);

%Kalman Filter

for i = 100:m
    [ XHat_New , estVar_New ] = kalmanF(i, xNewCords , XHat_Old , estVar_Old );
    XHat_Old = XHat_New;
    estVar_Old = estVar_New ;
    xCord_KalPos(i)  = XHat_New(1);
    xCords_KalVel(i) = XHat_New(2);
    xCords_KalAcc(i) = XHat_New(3);
  
end
%{
for i = 100:m
    [yHat10 , estVar10 , yHat00 , estVar00 ] = kalmanF(i, yNewCords , yHat00 , estVar00  );
    yCordKal(i,1) = yHat10(1,1);
    yCordKal(i,2) = yHat10(2,1);
end

for i = 100:m
    [zHat10 , estVar10 , xHat00 , estVar00 ] = kalmanF(i, zNewCords , zHat00 , estVar00  );
    zCordKal(i,1) = zHat10(1,1);
    zCordKal(i,2) = zHat10(2,1);
end

for i = 100:m
    [zHat10 , estVar10 , xHat00 , estVar00 ] = kalmanF(i, zNewCords , zHat00 , estVar00  );
    zCordKal(i,1) = zHat10(1,1);
    zCordKal(i,2) = zHat10(2,1);
end

for i = 100:m
    [zHat10 , estVar10 , xHat00 , estVar00 ] = kalmanF(i, zNewCords , zHat00 , estVar00  );
    zCordKal(i,1) = zHat10(1,1);
    zCordKal(i,2) = zHat10(2,1);
end

for i = 100:m
    [zHat10 , estVar10 , xHat00 , estVar00 ] = kalmanF(i, zNewCords , zHat00 , estVar00  );
    zCordKal(i,1) = zHat10(1,1);
    zCordKal(i,2) = zHat10(2,1);
end
%}
