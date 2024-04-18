%% Problem formulation
function [problem] = VanDerPol
% Van der Pol problem with state constraints
%
% Syntax:  [problem] = VanDerPol
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
problem.time.tf = 5;

% Number of states.
problem.nx = 2;

% Number of inputs.
problem.nu = 1;

% Initial conditions for system. Bounds if x0 is free s.t. x0l=< x0 <=x0u
% If fixed, x0l == x0u
problem.states.x0l = [1 0]; 
problem.states.x0u = [1 0]; 

% State bounds. xl=< x <=xu
limit = -0.4;
problem.states.xl = [-inf limit];
problem.states.xu = [inf inf];

% Terminal state bounds. xfl=< xf <=xfu. If fixed: xfl == xfu
problem.states.xfl = [-inf -inf]; 
problem.states.xfu = [inf inf];

% Input bounds
problem.inputs.ul = -inf;
problem.inputs.uu = inf;

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
dx2 = (1-x(1)^2)*x(2)-x(1)+u(1);
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

u1 = u(1);
lag = x(1)^2+x(2)^2+u1^2;

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
