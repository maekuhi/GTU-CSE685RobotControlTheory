% closedloop differential equation for adaptive control
% by M. Amani E. 
% https://sites.google.com/view/maekuhi
function dx = adaptiveODE(t,x,thetaTrue, Lambda, Kr, Gamma, thetaLimits)
%% State, State dot, estim.
q = x(1:3);
dq = x(4:6);
thetaHat = x(7:15);

[tau, dthetaHat] = adaptiveControl(t, q, dq, thetaHat, ...
    Lambda, Kr, Gamma, thetaLimits);

[M, Vm, Fd] = robotMatrices(q, dq, thetaTrue);
ddq = M \ (tau - Vm*dq - Fd*dq);

dx = [dq; ddq; dthetaHat];





end
