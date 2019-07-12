function [y_new] = lp_Filter(y_old,x_old,alpha_LP)

    y_new = alpha_LP*x_old + (1 - alpha_LP)*y_old ;

end