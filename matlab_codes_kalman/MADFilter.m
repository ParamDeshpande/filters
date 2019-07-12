function [outlier,medianVal] = MADFilter(testValIndex,col,matrix,order)
A = zeros(order,1);
B = zeros(order,1);

for i = 1:order
   A(i,1) = [matrix((testValIndex-order+i),col)];
end
medianVal = median(A);
for j = 1:order
    B(j,1) = abs(medianVal-A(j,1));
end
madValue = median(B);
    
     if(B(end)>(3*madValue))
      outlier = 1;
    else
      outlier = 0;
     end
end