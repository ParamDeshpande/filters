function [y_new] = hp_Filter(y_old,x_new,x_old,alpha_HP)
    y_new = (alpha_HP)*(y_old + x_new - x_old);
end