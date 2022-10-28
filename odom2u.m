function u = odom2u(odom0, odom1)
%UNTITLED8 Summary of this function goes here
%   Detailed explanation goes here
x0 = odom0(1);
y0 = odom0(2);
th0 = odom0(3);

x1 = odom1(1);
y1 = odom1(2);
th1 = odom1(3);

rot1 = atan2(y1 - y0, x1 - x0) - th0;
tran = ((x0 - x1)^2 + (y0 - y1)^2)^0.5;
rot2 = th1 - th0 - rot1;
u = [tran, rot1, rot2];

end