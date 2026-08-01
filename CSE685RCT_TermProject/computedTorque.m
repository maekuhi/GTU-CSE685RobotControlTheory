% computed-torque control law.
% by M. Amani E. 
% https://sites.google.com/view/maekuhi
function tau = computedTorque(t, q, dq,thetaModel,Kp,Kd)
[qd, dqd, ddqd] = desiredTrajectory(t);
e = qd - q;
de = dqd - dq;
[M, Vm, Fd] = robotMatrices(q, dq, thetaModel);
v = ddqd + Kd*de + Kp*e;

tau = M*v + Vm*dq + Fd*dq;

end
