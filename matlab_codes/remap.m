%   Limits and corrects output from 0 to 2pi

function y = remap(x)
    if(x > pi)
        y = x - 2*pi;
    elseif(x< -pi)
        y = x + 2*pi;
    else
        y = x;
    end
    
end
