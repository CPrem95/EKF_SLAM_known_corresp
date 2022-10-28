function [P] = gaussian(mean, std, z)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
var = std^2;
P = ((2*pi*var)^(-0.5))*exp(-0.5/var*(z - mean).^2);
figure();
plot(z, P);
end