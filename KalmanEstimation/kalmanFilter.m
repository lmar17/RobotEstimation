function [xpk1, xfk, param] = kalmanFilter(dt, yk, xpk, param)
% Kalman Filter for tracking a robot in 2D space.
% dt: time interval t(k) - t(k-1)
% yk: measurements at time k, y[k]
% xpk1: apriori estimate of state at k+1, x[k+1]
% param: various KF paramaters

A = param.A;
C = param.C;
P = param.P;
Q = param.Q;
R = param.R;

%%% KF updates
% compute a posteriori values using measument updates
K = P*C' * inv(C*P*C' + R);
P = P - K*C*P;
xfk = xpk + K*(yk - C*xpk);

% compute a priori estimates at k+1
xpk1 = A*xfk;
param.P = A*P*A' + Q;

end
