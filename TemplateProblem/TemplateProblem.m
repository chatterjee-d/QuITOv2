%% Problem formulation
function [problem] = TemplateProblem
% Template problem
%
% Syntax:  [problem] = TemplateProblem
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

% Settings file
problem.settings = @settings;

% Initial time. t0<tf. NOTE: t_0 has to be zero.
problem.time.t0 = 0; 

% Final time. tf is fixed.
problem.time.tf = FinalTime;

% Number of states.
problem.nx = TotalNumberOfStates;

% Number of inputs.
problem.nu = TotalNumberOfControlInputs;

% Initial conditions for system. Bounds if x0 is free s.t. x0l=< x0 <=x0u
% If fixed, x0l == x0u
problem.states.x0l = [x0l_1 x0l_2 ...]; % Lower bound on initial state
problem.states.x0u = [x0u_1 x0u_2 ...]; % Upper bound on initial state

% State bounds. xl=< x <=xu
problem.states.xl = [xl_1 xl_2 ...]; % Lower bound on state
problem.states.xu = [xu_1 xu_2 ...]; % Upper bound on state

% Terminal state bounds. xfl=< xf <=xfu. If fixed: xfl == xfu
problem.states.xfl = [xfl_1 xfl_2 ...]; % Lower bound on final state
problem.states.xfu = [xfu_1 xfu_2 ...]; % Upper bound on final state

% Input bounds
problem.inputs.ul = [ul_1 ul_2 ...]; % Lower bound on control
problem.inputs.uu = [uu_1 uu_2 ...]; % Upper bound on control

% Bounds on the first control action
problem.inputs.u0l = [-inf -inf ...]; % Lower bound on initial control
problem.inputs.u0u = [inf inf ...]; % Upper bound on initial control

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
dx2 = u(1);
dx3 = 0.5 * (x(2)^2 - x(1)^2);
dx = [dx1; dx2; dx3];

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

lag = x(1)^2 + u(2)^2;

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

mayer = x(3)^2;

%-------------- END CODE ---------------
end
