function [true_path, nSamples, plt] = path_plan(start, stop, n, type)
%UNTITLED5 Summary of this function goes here
%   Detailed explanation goes here
switch type
    case 'simple'
        x = linspace(start(1), stop(1), n);
        y = 0.5* (x + 0).^1.2 ;

    case 'comp'
        % Specify waypoints, times of arrival, and sampling rate.
        wp = [0 0 0; 2 -2 0; 4 0 0; 6 2 0; 8 0 0; 6 -2 0; 4 0 0; 2 2 0; 0 0 0];
        toa = 4*(0:size(wp,1)-1).';
        Fs = 100;
        % Create trajectory.
        traj = waypointTrajectory(wp, toa, SampleRate=Fs);
        % Get position.
        t = 0:n-1;
        pos = lookupPose(traj, t);

        x = pos(:,1)';
        y = pos(:,2)';

    case 'linear_x'
        x = linspace(start(1), stop(1), n);
        y = 0* x;

    case 'spiral'
        % given values
        pos = [stop(1) stop(2) ;    % stoppoint
            start(1), start(2) ] ;  % startpoint
        nturns = 1 ;    % number of turns (integer value)
        % engine
        dp = diff(pos,1,1) ;
        R = hypot(dp(1), dp(2)) ;
        phi0 = atan2(dp(2), dp(1)) ;
        phi = linspace(0, nturns*2*pi, n) ; % 10000 = resolution
        r = linspace(0, R, numel(phi)) ;
        x = pos(1,1) + r .* cos(phi + phi0) ;
        y = pos(1,2) + r  .* sin(phi + phi0) ;
        x = flip(x);
        y = flip(y);
plot(x,y,'b-',pos(:,1),pos(:,2),'ro-') ;
end
th = zeros(1,n);

for i = 2:n
    th(i) = atan2(y(i) - y(i-1), x(i) - x(i-1));
end

% for i = 1:n
%     if y(i) > stop(2)
%         break
%     end
% end
x = x(1:i-1);
y = y(1:i-1);
th = th(1:i-1);

true_path = [x; y; th]';
nSamples = height(true_path);

plt = scatter(x,y, 'Color', '#EDB120');

r = 0.2;
for i = 1:nSamples
    X = [x(i), x(i)+r*cos(th(i))];
    Y = [y(i), y(i)+r*sin(th(i))];
    line(X, Y, 'Color', '#EDB120');
    hold on
end
end