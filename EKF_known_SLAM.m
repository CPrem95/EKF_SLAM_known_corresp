clc
close all
clear all

%% Setting up the Environment and the Robot
% Environment
env_lims = [12, 10]; % [xlimit, ylimit]
n_landm = 20;

% Robot path
path_start = [0, 0];
path_stop = [11, 10];
exp_sample_n = 200; % number of expected samples in the range

% Robot
rad_range = 6;

fig = figure();
set(fig,'defaultLegendAutoUpdate','off');
figure()

[env, plt1] = create_env(n_landm, env_lims, 'rand'); % 'predef' or 'rand'
hold on
% load('env')
plt1 = scatter(env(:,1), env(:,2), '*');

[path, path_samples, plt2] = path_plan(path_start, path_stop, exp_sample_n, 'simple'); % 'simple' or 'comp'
hold on

plt3 = scatter(NaN, NaN, 'r');
plt4 = scatter(NaN, NaN, 'k');
plt5 = scatter(NaN, NaN, 'magenta');
plt6 = line(NaN, NaN,'Color','#00FFFF');
%
legend([plt1, plt2, plt3, plt4, plt5, plt6],{'Landmarks', 'True path', 'Odometry', 'Estimated path', 'Estimated Landmarks', 'Observations'}, 'AutoUpdate','off');
xlim([-2 12])
ylim([-3 12])
[obs ,odom] = robot(env, path, rad_range, false);
hold on
pause(1)
% load('obs.mat')
% load('odom.mat')
%% Initialization
% mean and the covariance matrix
ini_landm_var = 1000000;

% mapping function (from low dimensional space to high dimensional space)
F = [eye(3), zeros(3, 2*n_landm)];

ini_mu = zeros(3 + n_landm*2, 1);

ini_cov_xx = zeros(3, 3);
ini_cov_xm = zeros(3, n_landm*2);
ini_cov_mx = zeros(n_landm*2, 3);
ini_cov_mm = eye(n_landm*2, n_landm*2)*ini_landm_var;

ini_cov = zeros(3 + n_landm*2, 3 + n_landm*2);
ini_cov(1:3, 1:3) = ini_cov_xx;
ini_cov(1:3, 4:end) = ini_cov_xm;
ini_cov(4:end, 1:3) = ini_cov_mx;
ini_cov(4:end, 4:end) = ini_cov_mm;

% ini_cov

%% Prediction Step
% Use the mapping function F
% Calcutlate predicted mean mu
% Calculate the Jacobian G
% Calculate the expected covariance sig

%
sig = ini_cov;
mu = ini_mu;

% Motion uncertainty
R = [0.1, 0, 0;
    0, 0.1, 0;
    0, 0, 0.1];

% Observation uncertainty
Q = [0.25 0;
    0 0.1];

est_landm = scatter(0,0); % needed for the visualization in next part
%% Main Part (Online loop)
for i = 2:path_samples
    %% Main Loop
    disp(i)
    u = odom2u(odom(i -1, :), odom(i, :));
    [mu_bar, sig_bar] = EKF_known_predict(mu, sig, u, R, F, n_landm);

    [mu, sig] = EKF_known_correction(mu_bar, sig_bar, obs(i,:,:), Q, n_landm, ini_landm_var);
    disp('-------------------------------')
    %     disp(sig)
    %% Visualization
    % Online observation plotting
    lines = [];
    obs_pos = [];
    landm_ellipses = [];

    scatter(odom(i, 1), odom(i, 2), 'blue')
    for k = 1:n_landm
        X = [mu(1), mu(1)+obs(i, k, 1)*cos(obs(i, k, 2) + mu(3))];
        Y = [mu(2), mu(2)+obs(i, k, 1)*sin(obs(i, k, 2) + mu(3))];
        L = line(X, Y, 'Color', '#00FFFF');
        o_p = scatter(mu(1)+obs(i, k, 1)*cos(obs(i, k, 2) + mu(3)), mu(2)+obs(i, k, 1)*sin(obs(i, k, 2) + mu(3)), 5, 'red', 'filled');
        if mu(2+2*k: 3+2*k) ~= [0,0] % ellipses only if there's an observation
            l_e = error_ellipse(sig(2+2*k: 3+2*k, 2+2*k: 3+2*k), mu(2+2*k: 3+2*k), 'style', 'cyan');
        else
            l_e = error_ellipse([0.001,0; 0,0.001], mu(2+2*k: 3+2*k));
        end
        lines = [lines, L];
        obs_pos = [obs_pos, o_p];
        landm_ellipses = [landm_ellipses, l_e];
        %         lm = [lm, obs(i, k, 1)];
        hold on
    end

    est_landm.Visible = "off";
    est_landm = scatter(mu(4:2:end), mu(5:2:end),'m');
    %     disp(lm)

    scatter(mu(1), mu(2), 'k')
    hold on

    r = 0.2;
    lX = [mu(1), mu(1)+r*cos(mu(3))];
    lY = [mu(2), mu(2)+r*sin(mu(3))];
    line(lX, lY, 'Color', 'k');
    hold on
    pose_ellipse = error_ellipse(sig(1:2, 1:2), mu(1:2), 'style', 'green');
    pause(0.01)
    % Removing all the annotations of the observation
    for j = 1:n_landm
        lines(j).Visible = 'off';
        obs_pos(j).Visible = 'off';
        pose_ellipse.Visible = 'off';
        landm_ellipses(j).Visible = 'off';
    end
end

