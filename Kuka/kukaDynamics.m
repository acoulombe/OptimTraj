function dx = kukaDynamics(x, u, robot)
% This function computes the dynamics of the Kuka iiwa 7 manipulator
%
% For when things start breaking : https://www.mathworks.com/help/robotics/referencelist.html?type=function&listtype=cat&category=index&blocktype=all&capability=
%
% INPUTS:
%   x = [14,N] = state vector
%   u = [7,N] = input torque
%   robot = rigidBodyTree of Kuka iiwa 7
%
% OUTPUTS:
%   dx = [14,N] = dx/dt = time derivative of the state
% 
% NOTES:
%   
%   states:
%       1 = q1 = first link angle
%       2 = q2 = second link angle
%       3 = q3 = third link angle
%       4 = q4 = fourth link angle
%       5 = q5 = fifth link angle
%       6 = q6 = sixth link angle
%       7 = q7 = seventh link angle
%       8 = dq1 = first link angular rate
%       9 = dq2 = second link angular rate
%       10 = dq3 = third link angular rate
%       11 = dq4 = fourth link angular rate
%       12 = dq5 = fifth link angular rate
%       13 = dq6 = sixth link angular rate
%       14 = dq7 = seventh link angular rate
%

dx = [];
N = size(u, 2);

if N == 1
   % Split state into joint position and velocities
    q = x(1:7);
    dq = x(8:14);
    
    % Perform Forward Dynamics of the system
    ddq = forwardDynamics(robot, q, dq, u);

    dx = [dq; ddq];

else
    for i=1:N
        % Split state into joint position and velocities
        q = x(1:7, i);
        dq = x(8:14, i);

        % Perform Forward Dynamics of the system
        ddq = forwardDynamics(robot, q, dq, u(:,i));

        dx = [dx, [dq; ddq]];
    end
end

end