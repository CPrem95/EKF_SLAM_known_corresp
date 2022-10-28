function [mu_bar, sig_bar] = EKF_known_predict(mu, sig, u, R, F, n)
%mu = current state estimate
%sig = current state uncertainty
%u = input(odom) = [tran, rot1, rot2]
%R = Motion uncertainty
%F = Mapping function
%n = Number of landmarks
% Use the mapping function F
% Calcutlate predicted mean mu
% Calculate the Jacobian G
% Calculate the expected covariance sig

tran = u(1);
rot1 = u(2);
rot2 = u(3);

% velocity based motion model
odo = [tran*cos(mu(3) + rot1);
       tran*sin(mu(3) + rot1);
       rot1 + rot2];
% the velocity commands update only the pose of the robot
% prdicted mean of the state
mu_bar =  mu + F'*odo;

% the Jacobian of the motion model affects only the robot's motion
% The map remains unchanged (multiplied by identity)
g = [0, 0, -tran*sin(mu(3) + rot1);...
     0, 0, tran*cos(mu(3) + rot1);...
     0, 0, 0];

% Compute the Jacobian
G = eye(3 + 2*n) + F'*g*F;

% Update covariance
sig_bar = G*sig*G' + F'*R*F;

end