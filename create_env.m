function [env, plt] = create_env(n, area, type)
%Creates virtual landmarks
%   n = number of landmarks
%   area corresponds to [xlimit, ylimit] of the environment
limx = area(1);
limy = area(2);

switch type
    %r = -5 + (5+5)*rand(10,1) >>>> interval (-5, 5)
    case 'rand'
        x = limx*rand(n, 1);
        y = limy*rand(n, 1);

    case 'predef1'
        x = linspace(0.5,limx - 0.5,n/2)';
        y = 2* ones(length(x), 1);

        x = [x; linspace(0.5,limx - 0.5,n/2)'];
        y = [y; 5* ones(length(x)/2, 1)];

    case 'predef2'
        x = linspace(0.5,limx - 0.5,n)';
        y = 1.5* ones(length(x), 1);

end
plt = scatter(x, y, '*');
axis([0 limx+2 0 limy+2])
axis equal
env = [x, y];
end