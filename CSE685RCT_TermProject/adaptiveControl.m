% adaptive filtered-error controller & update law.
% by M. Amani E. 
% https://sites.google.com/view/maekuhi
function [tau, dthetaHat]=adaptiveControl(t, q, dq,thetaHat,Lambda, Kr, Gamma, thetaTrue)
%% e and e dot
[qd, dqd, ddqd] = desiredTrajectory(t);
e = qd - q;
de =dqd - dq;
%% filter
r = de + Lambda*e;
dqRef = dqd + Lambda*e;
ddqRef = ddqd + Lambda*de;
%% regress.
Y = regressorMatrix(q, dq, dqRef, ddqRef);
tau = Y*thetaHat + Kr*r;
dthetaHat = Gamma*Y'*r;

% keeping the estimates positive and reasonable numerical ranges.
lowerLimit = 0.1*thetaTrue;
upperLimit = 2.0*thetaTrue;

for i = 1:9
    if thetaHat(i) <= lowerLimit(i) && dthetaHat(i) < 0
        dthetaHat(i) = 0;
    elseif thetaHat(i) >= upperLimit(i) && dthetaHat(i) > 0
        dthetaHat(i) = 0;
    end
end

end

