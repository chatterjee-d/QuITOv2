%% Problem formulation
function [problem] = RayleighProblemMixedConstraint
% Rayleigh problem with mixed constraints
%
% Syntax:  [problem] = RayleighProblemMixedConstraints
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
problem.time.tf = 4.5;

% Number of states.
problem.nx = 2;

% Number of inputs.
problem.nu = 1;

% Initial conditions for system. Bounds if x0 is free s.t. x0l=< x0 <=x0u
% If fixed, x0l == x0u
problem.states.x0l = [-5 -5]; % Lower bound on initial state
problem.states.x0u = [-5 -5]; % Upper bound on initial state

% State bounds. xl=< x <=xu
problem.states.xl = [-inf -inf]; % Lower bound on state
problem.states.xu = [inf inf]; % Upper bound on state

% Terminal state bounds. xfl=< xf <=xfu. If fixed: xfl == xfu
problem.states.xfl = [-inf -inf]; % Lower bound on final state
problem.states.xfu = [+inf inf]; % Upper bound on final state

% Input bounds
problem.inputs.ul = [-inf]; % Lower bound on control
problem.inputs.uu = [inf]; % Upper bound on control

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

dx1 = x(2);
dx2 = -x(1) + x(2) * (1.4 - 0.14*x(2)^2) + 4 * u(1);
dx = [dx1; dx2];

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

lag = x(1)^2 + u(1)^2;

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
