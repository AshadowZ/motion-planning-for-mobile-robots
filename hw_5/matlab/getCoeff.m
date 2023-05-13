function coeff = getCoeff(t)
% Get the coefficient for Aeq，对应课件中的A_j矩阵
    coeff = [1,  1*t,  1*t^2,  1*t^3,  1*t^4,  1*t^5,  1*t^6,  1*t^7; % k=0
             0,  1,    2*t,    3*t^2,  4*t^3,  5*t^4,  6*t^5,  7*t^6; % k=1
             0,  0,    2,      6*t,    12*t^2, 20*t^3, 30*t^4, 42*t^5; % k=2
             0,  0,    0,      6,      24*t,   60*t^2, 120*t^3,210*t^4]; % k=3
end