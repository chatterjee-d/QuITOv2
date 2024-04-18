%% Problem formulation
function [problem] = siri
% Aly Chan problem
%
% Syntax:  [problem] = AlyChan
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
problem.time.tf = 20;

% Number of states.
problem.nx = 3;

% Number of inputs.
problem.nu = 3;

% Initial conditions for system. Bounds if x0 is free s.t. x0l=< x0 <=x0u
% If fixed, x0l == x0u
problem.states.x0l = [0.2 0.8 0]; 
problem.states.x0u = [0.2 0.8 0]; 

% State bounds. xl=< x <=xu
problem.states.xl = [-inf 0 0];
problem.states.xu = [+inf 1 1];

% Terminal state bounds. xfl=< xf <=xfu. If fixed: xfl == xfu
problem.states.xfl = [-inf 0 0]; 
problem.states.xfu = [+inf 1 1];

% Input bounds
problem.inputs.ul = [0.1 0.1 0];
problem.inputs.uu = [1 1 0.9];
%-------------- END CODE ---------------
end


%% System dynamics
function [dx] = dynamics(x,u,t)
cs = 2; cr = 2; ci = 5; cv = 3; beta = 0.7; beta_hat = 0.5; gamma = 0.5; % 0.5 works
dx1 = cs * x(2) + cr * x(3) + ci * ( 1 - x(2) - x(3) ) - cs * x(2) * u(1) - cr * x(3) * u(2) + cv * x(2) * u(3);
dx2 = - beta * x(2) * ( 1 - x(2) - x(3) ) * u(1) - x(2) * u(3);
dx3 = gamma * ( 1 - x(2) - x(3) ) - beta_hat * x(3) * ( 1- x(2) - x(3) ) * u(2) + x(2) * u(3);
dx = [dx1; dx2; dx3];
end

%% Cost functions

function lag = stageCost(x,u,t)
lag = 0;
end
function mayer = terminalCost(x,u,t)
mayer = x(1);
end
