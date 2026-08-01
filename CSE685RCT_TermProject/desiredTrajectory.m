% desired trajectory & its first two derivatives.
% by M. Amani E. 
% https://sites.google.com/view/maekuhi
function [qd, dqd, ddqd] = desiredTrajectory(t)

% according to proj. descr.
a = [0.7; 1.2; 1.6];

h = 1 - exp(-0.3*t^3);
dh = 0.9*t^2*exp(-0.3*t^3);
ddh = (1.8*t - 0.81*t^4)*exp(-0.3*t^3);

qd = a*sin(t)*h;
dqd = a*(cos(t)*h + sin(t)*dh);
ddqd = a*(-sin(t)*h + 2*cos(t)*dh + sin(t)*ddh);





end

