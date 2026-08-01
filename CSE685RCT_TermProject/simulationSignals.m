% ccalculate tracking errors and controller inputs after an ODE simulation
% by M. Amani E. 
% https://sites.google.com/view/maekuhi
function [error, filteredError, torque] = simulationSignals(t, x, ...
    controllerNumber, thetaModel, Kp, Kd, Lambda, Kr, thetaBound, phi)


n = length(t);
error = zeros(n,3);
filteredError = zeros(n,3);
torque = zeros(n,3);

for i = 1:n
    q = x(i,1:3)';
    dq = x(i,4:6)';
    [qd, dqd] = desiredTrajectory(t(i));
    error(i,:) = (qd - q)';
    de = dqd - dq;
    filteredError(i,:) = (de + Lambda*(qd-q))';

    if controllerNumber == 1
        tau = computedTorque(t(i), q, dq, thetaModel, Kp, Kd);
    elseif controllerNumber == 2
        tau = robustControl(t(i), q, dq, thetaModel, ...
            Lambda, Kr, thetaBound, phi);
    else
        thetaHat = x(i,7:15)';
        tau = adaptiveControl(t(i), q, dq, thetaHat, ...
            Lambda, Kr, eye(9), thetaModel);
    end

    torque(i,:) = tau';

    %
end



end
