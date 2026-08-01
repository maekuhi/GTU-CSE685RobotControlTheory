% closed-loop differential equation for robust control.
% by M. Amani E. 
% https://sites.google.com/view/maekuhi
function dx = robustODE(t, x, thetaTrue, thetaModel, Lambda, Kr, ...
    thetaBound, phi)

% modified ==

% ==


q = x(1:3);
dq = x(4:6);

tau = robustControl(t, q, dq, thetaModel, Lambda, Kr, ...
    thetaBound, phi);
[M, Vm, Fd] = robotMatrices(q, dq, thetaTrue);
ddq = M \ (tau - Vm*dq - Fd*dq);

dx = [dq; ddq];
end
