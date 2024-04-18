%% Problem formulation
function [problem] = invertedPendulumOnCart
% Inverted Pendulum on a cart with a mass spring system
%
% Syntax:  [problem] = invertedPendulumOnCart
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
problem.time.tf = 15;

% Number of states.
problem.nx = 6;

% Number of inputs.
problem.nu = 2;

% Initial conditions for system. Bounds if x0 is free s.t. x0l=< x0 <=x0u
% If fixed, x0l == x0u
problem.states.x0l = [0 pi/10 0 0 1 1.05]; 
problem.states.x0u = [0 pi/10 0 0 1 1.05]; 

% State bounds. xl=< x <=xu
problem.states.xl = [-2 -2 -2 -2 -2 -2];
problem.states.xu = [2 2 2 2 2 2];

% Terminal state bounds. xfl=< xf <=xfu. If fixed: xfl == xfu
problem.states.xfl = [0 0 0 0 0 0]; 
problem.states.xfu = [0 0 0 0 0 0];

% Input bounds
problem.inputs.ul = [-1 -1];
problem.inputs.uu = [1 1];

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
% Model equations
m = 0.25;
M = 3;
g = 9.81;
L = 5;
A = [0 1 0 0 0 0;
     0 0 -m*g/M 0 0 0;
     0 0 0 1 0 0;
     0 0 (m+M)*g/(L*M) 0 0 0;
     0 0 0 0 0 1;
     0 0 0 0 -1 0];
 B = [0 0;
      1/M 0.1;
      0 0;
      -1/(L*M) 0.1;
      0 0;
      0.1 1];

dx = A*x + B*u ;

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

Q = eye(6);
R = 0.1*eye(2);
lag = x'*Q*x + u'*R*u;

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
