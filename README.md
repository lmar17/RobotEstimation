# RobotEstimation
Robot estimation and learning using Bayesian learning for localization
and mapping.

This is a set of octave scripts for:
* KalmanEstimation: estimate robot position using Kalman Filter. Use
  Octave to run script robotTrack.m where noisy measurements of robot
  pose is filtered using KF. The robot motion is modeled with a simple
  linear model.

* Localization: estimate robot position using range sensor data (LIDAR).
