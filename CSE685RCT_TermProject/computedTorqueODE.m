% closed-loop differential equation for computed-torque control
% by M. Amani E. 
% https://sites.google.com/view/maekuhi
function dx = computedTorqueODE(t, x, thetaTrue, thetaModel, Kp, Kd)




q = x(1:3);
dq = x(4:6);

tau = computedTorque(t,q,dq, thetaModel, Kp, Kd);

[M, Vm, Fd] = robotMatrices(q, dq, thetaTrue);

ddq = M \ (tau - Vm*dq - Fd*dq);

dx = [dq; ddq];

end
