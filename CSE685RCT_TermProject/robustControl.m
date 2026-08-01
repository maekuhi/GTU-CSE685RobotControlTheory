% robust filtered-error control law
% by M. Amani E. 
% https://sites.google.com/view/maekuhi
function tau = robustControl(t, q, dq, thetaModel, Lambda, Kr, ...
    thetaBound, phi)

[qd, dqd, ddqd] = desiredTrajectory(t);
e = qd - q;
de = dqd - dq;

r = de + Lambda*e;
dqRef = dqd + Lambda*e;
ddqRef = ddqd + Lambda*de;

[M, Vm, Fd] = robotMatrices(q, dq, thetaModel);

% according to the lecture and references it is given that: |Y*thetaError| <= |Y|*thetaBound.
Y = regressorMatrix(q, dq, dqRef, ddqRef);
rho = abs(Y)*thetaBound;

% ** componentwise saturation is used instead of sign(r) to reduce chattering
satValue = max(-1, min(1, r/phi));
robustTerm = rho .* satValue;

tau = M*ddqRef + Vm*dqRef + Fd*dq + Kr*r + robustTerm;
end
