%% Problem formulation
function [problem] = RobotPathPlanning
% Robot Path Planning problem
%
% Syntax:  [problem] = RobotPathPlanning
%
% Outputs:
%    problem - Structure with information on the optimal control problem

%------------- BEGIN CODE --------------
% Set system dynamics
problem.dynamicsFunc = @dynamics;

% Set Lagrange cost (Stagewise cost) to be minimized
problem.stageCost = @stageCost;

% Set Mayer cost (Terminal cost)
problem.terminalCost = @terminalCost;

% Initial time. t0<tf. NOTE: t_0 has to be zero.
problem.time.t0 = 0; 

% Final time. tf is fixed.
problem.time.tf = 10;

% Number of states.
problem.nx = 4;

% Number of inputs.
problem.nu = 2;

% Initial conditions for system. Bounds if x0 is free s.t. x0l=< x0 <=x0u
% If fixed, x0l == x0u
problem.states.x0l = [40 5 -inf -inf]; 
problem.states.x0u = [40 5 inf inf]; 

% State bounds. xl=< x <=xu
problem.states.xl = [0 0 -10 -10];
problem.states.xu = [80 80 10 10];

% Terminal state bounds. xfl=< xf <=xfu. If fixed: xfl == xfu
problem.states.xfl = [55 70 -inf -inf]; 
problem.states.xfu = [55 70 inf inf];

% Input bounds
problem.inputs.ul = [-inf -inf];
problem.inputs.uu = [inf inf];

%-------------- END CODE ---------------
end


%% System dynamics
function [dx] = dynamics(x,u,t)
% Problem dynamics
%
% Syntax:  
%          [dx] = Dynamics(x,u,t)	(Dynamics Only)
% 
% Inputs:
%    x  - state vector
%    u  - input vector
%    t  - time
%
% Output:
%    dx - time derivative of x
%
%------------- BEGIN CODE --------------

dx1 = x(3);
dx2 = x(4);
dx3 = u(1);
dx4 = u(2);
dx = [dx1; dx2; dx3; dx4];

%-------------- END CODE ---------------
end


%% Cost functions

function lag = stageCost(x,u,t)
% Lagrange cost to be minimized
%
% Syntax:  
%          [lag] = stageCost(x,u,t)	(stageCost Only)
% 
% Inputs:
%    x  - state vector
%    u  - input
%    t  - time
%
% Output:
%    lag - stage wise cost (lagrange cost)
%
%------------- BEGIN CODE --------------

lag = (u(1)^2) + (u(2)^2);

%-------------- END CODE ---------------
end


function mayer = terminalCost(x,u,t)
% Mayer cost to be minimized
%
% Syntax:  
%          [mayer] = terminalCost(x,u,t)
% 
% Inputs:
%    x  - state vector
%    u  - input
%    t  - time
%
% Output:
%    mayer - terminal cost (mayer cost)
%
%------------- BEGIN CODE --------------

mayer = 0;

%-------------- END CODE ---------------
end
