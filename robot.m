function [obs ,odom] = robot(env, path, r_thresh, vis)
%This func provides observations and odometry readings of the robot
%   'env' has the output of create_env
%   'path' has the output of path_plan (true path)
%   'r_thresh' is the radar range
%   'vis' to enable visualization

% True Path
u = path; % The path acts as the input commands: u
n_odom = height(u);
n_lm = height(env);
obs = zeros(n_odom, n_lm, 3);

sum_err_x = 0;
sum_err_y = 0;
sum_err_th = 0;

for i = 2:n_odom
    sum_err_x = sum_err_x + 1*normrnd(0.05, 0.01);
    sum_err_y = sum_err_y - 1*normrnd(0.01, 0.002);
    sum_err_th = sum_err_th + 1*normrnd(0, 0.01);

    u(i, 1) = u(i, 1) + sum_err_x;
    u(i, 2) = u(i, 2) + sum_err_y;
    u(i, 3) = u(i, 3) + sum_err_th;
end
odom = u;

% Observations
for i = 1:n_odom
    for k = 1:n_lm
        r = ((env(k, 1) - path(i, 1))^2 + (env(k, 2) - path(i, 2))^2)^0.5;
        th = atan2(env(k, 2) - path(i, 2), env(k, 1) - path(i, 1)) - path(i, 3);
        if r < r_thresh
            obs(i, k, 1) = r + 0*normrnd(0, 0.1);
            obs(i, k, 2) = th + 0*normrnd(0, 0.03); % 0.1 rad = 1.71 deg
            obs(i, k, 3) = k;
        end
    end
end

if vis == true;
    %Visualizations
    r = 0.2;
    for i = 1:n_odom
        % Plotting the odometry data
        scatter(odom(i, 1), odom(i, 2), 'r')
        X = [odom(i,1), odom(i,1) + r*cos(odom(i,3))];
        Y = [odom(i,2), odom(i,2) + r*sin(odom(i,3))];
        line(X, Y, 'Color', 'r');
        hold on

        % Plotting the observations
        lines = [];
        obs_pos = [];
        %     lm = [];
        for k = 1:n_lm
            X = [path(i,1), path(i,1)+obs(i, k, 1)*cos(obs(i, k, 2) + path(i, 3))];
            Y = [path(i,2), path(i,2)+obs(i, k, 1)*sin(obs(i, k, 2) + path(i, 3))];
            L = line(X, Y, 'Color', '#00FFFF');
            o_p = scatter(path(i,1)+obs(i, k, 1)*cos(obs(i, k, 2)  + path(i, 3)), path(i,2)+obs(i, k, 1)*sin(obs(i, k, 2) + path(i, 3)), 5, 'red', 'filled');
            lines = [lines, L];
            obs_pos = [obs_pos, o_p];
            %         lm = [lm, obs(i, k, 1)];
            hold on
        end
        %     disp(lm)
        pause(0.1)
        % Removing all the annotations of the observation
        for j = 1:n_lm
            lines(j).Visible = 'off';
            obs_pos(j).Visible = 'off';
        end
    end
end
end

