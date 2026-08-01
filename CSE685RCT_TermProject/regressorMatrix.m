% build the regressor one parameter at a time.
% by M. Amani E. 
% https://sites.google.com/view/maekuhi
function Y = regressorMatrix(q, dq, dqRef, ddqRef)



Y = zeros(3,9);

for i = 1:9
    oneParameter = zeros(9,1);
    oneParameter(i) = 1;

    [M, Vm, Fd] = robotMatrices(q, dq, oneParameter);
    Y(:,i) = M*ddqRef + Vm*dqRef + Fd*dq;
end

end
% !! This works because the robot model is linear in the nine parameters
