% Track a mobile robot 2D position (x, y) and robot coordinate frame
% orientation (theta) relative to the world frame.

% Load mobile robot data. The data is taken from the Coursera Robotics
% course Estimation and Learning by Prof. Dan Lee, University of
% Pennsylvania.

% Two variables are loaded: t, pose
%
% [1] t: is a Kx1 array with sample times in seconds, K=3701.
% [2] pose: is a 3xK array with the mobile robot pose over time,
% e.g. pose(:, k) is [x(meter); y(meter); theta(radian)] robot pose at
% sample k.
load robotPose.mat

% The robot motion is modeled as a linear dynamical system where x, y,
% theta state varables are independent and changing at a constant
% velocity. So the robot motion can be represented as three
% independent linear systems:
%
% Position x and velocity vx:
% sx[k] = {x[k] vx[k]}'
% sx[k+1] = Ax*sx[k]
% zx[k] = Cx*sx[k]
%
% Position y and velocity vy:
% sy[k] = {y[k] vy[k]}'
% sy[k+1] = Ay*sy[k]
% zy[k] = Cy*sy[k]
%
% Angular orientation theta and velocity vtheta:
% st[k] = {theta[k] vtheta[k]}'
% st[k+1] = At*stheta[k]
% zt[k] = Ct*stheta[k]

% linear model parameters
dt = 0;
paramX.A = [1, dt; 0, 1];
paramX.C = [1, 0];

paramY.A = [1, dt; 0, 1];
paramY.C = [1, 0];

paramT.A = [1, dt; 0, 1];
paramT.C = [1, 0];

% system and measurement Gaussian noise parameters
stdX = 0.01;
stdVx = 0.1;
stdMx = 0.05;
paramX.Q = [stdX^2, 0; 0, stdVx^2];
paramX.R = stdMx^2;

stdY = 0.01;
stdVy = 0.1;
stdMy = 0.05;
paramY.Q = [stdY^2, 0; 0, stdVy^2];
paramY.R = stdMy^2;

stdT = 0.01;
stdVt = 0.1;
stdMt = 0.01;
paramT.Q = [stdT^2, 0; 0, stdVt^2];
paramT.R = stdMt^2;

% add noise to pose measurements to simulate measurement errors
N = numel(t);
npose = pose + randn(3, N) .* [stdMx, stdMy, stdMt]';

% create and initialize filtered and predicted state
XFK = zeros(3, N);
XFK(:, 1) = pose(:, 1);
XPK = zeros(3, N+1);
XPK(:, 1) = pose(:, 1);
XPK(:, 2) = pose(:, 2);

% initialize state and state covariance
sx = [npose(1, 1), 0]';
sy = [npose(2, 1), 0]';
st = [npose(3, 1), 0]';
paramX.P = 10*eye(numel(sx));
paramY.P = 10*eye(numel(sx));
paramT.P = 10*eye(numel(sx));

% run KF
for ii = 2:N
    dt = t(ii) - t(ii-1);
    paramX.A = [1, dt; 0, 1];
    paramY.A = [1, dt; 0, 1];
    paramT.A = [1, dt; 0, 1];

    [sx, fx, paramX] = kalmanFilter(dt, npose(1, ii), sx, paramX);
    [sy, fy, paramY] = kalmanFilter(dt, npose(2, ii), sy, paramY);
    [st, ft, paramT] = kalmanFilter(dt, npose(3, ii), st, paramT);
    
    XFK(:, ii) = [fx(1), fy(1), ft(1)]';
    XPK(:, ii+1) = [sx(1), sy(1), st(1)]';
end

% plot the actual robot position
figure(1); clf;
plot(pose(1, :), pose(2, :), 'bo');
set(gca, 'YDir', 'reverse')
axis equal; hold on;
title('Robot Position', 'FontSize', 12, 'FontWeight', 'bold')
xlabel('X meters')
ylabel('Y meters')

% plot measured, filtered, and estimated position
plot(npose(1, :), npose(2, :), 'k.');
plot(XFK(1, :), XFK(2, :), 'g.');
plot(XPK(1, :), XPK(2, :), 'r.');
legend('Actual', 'Measured', 'Filtered', 'Estimated', 'Location', ...
       'northwest');

% mean and std of measured, filtered, and estimated pose error
error = sqrt((npose(1, :) - pose(1, :)).^2 + ...
             (npose(2, :) - pose(2, :)).^2);
fprintf('Measured Error: mean=%f std=%f\n', mean(error), std(error));

error = sqrt((XFK(1, 1:N) - pose(1, :)).^2 + ...
             (XFK(2, 1:N)- pose(2, :)).^2);
fprintf('Filtered Error: mean=%f std=%f\n', mean(error), std(error));

error = sqrt((XPK(1, 1:N) - pose(1, :)).^2 + ...
             (XPK(2, 1:N)- pose(2, :)).^2);
fprintf('Estimated Error: mean=%f std=%f\n', mean(error), std(error));
