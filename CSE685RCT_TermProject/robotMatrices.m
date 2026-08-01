% Matrices of the assigned 3-DOF robot.
% by M. Amani E. 
% https://sites.google.com/view/maekuhi
function [M, Vm, Fd] = robotMatrices(q, dq, theta)

b1 = theta(1);
b2 = theta(2);
b3 = theta(3);
p1 = theta(4);
p2 = theta(5);
p3 = theta(6);
% fd1 = theta(7) 
% fd2 = theta(8) 
% fd3 = theta(9)

q2 = q(2);
q3 = q(3);
dq1 = dq(1);
dq2 = dq(2);
dq3 = dq(3);

c2 = cos(q2);
c3 = cos(q3);
c23 = cos(q2 + q3);
s2 = sin(q2);
s3 = sin(q3);
s23 = sin(q2 + q3);

M = [b1 + 2*p1*c2 + p2*c23 + p3*c3, ...
     b2 + p1*c2 + p2*c23, ...
     b3 + p2*c23 + p3*c3;
     b2 + p1*c2 + p2*c23, ...
     b2 + 2*p3*c3, ...
     b3 + p3*c3;
     b3 + p2*c23 + p3*c3, ...
     b3 + p3*c3, ...
     b3];

a = p1*s2 + p2*s23;
b = p2*s23 + p3*s3;
d = p3*s3;

Vm = zeros(3);
Vm(1,1) = -a*dq2 - b*dq3;
Vm(1,2) = -a*(dq1 + dq2) - b*dq3;
Vm(1,3) = b*(-dq1 + dq2 + dq3);
Vm(2,1) = a*dq1 + d*dq3;
Vm(2,2) = -b*dq3;
Vm(2,3) = -d*(3*dq1 + dq2 + dq3);
Vm(3,1) = a*dq1 - d*dq2;
Vm(3,2) = d*(3*dq1 + dq2);

Fd = diag(theta(7:9));
end
