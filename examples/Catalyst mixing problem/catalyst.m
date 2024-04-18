%% Problem formulation
function [problem] = catalyst

problem.dynamicsFunc = @dynamics;

% Set Lagrange cost (Stagewise cost) to be minimized
problem.stageCost = @stageCost;

% Set Mayer cost (Terminal cost)
problem.terminalCost = @terminalCost;

% Initial time. t0<tf. NOTE: t_0 has to be zero.
problem.time.t0 = 0; 

% Final time. tf is fixed.
problem.time.tf = 4;

% Number of states.
problem.nx = 2;

% Number of inputs.
problem.nu = 1;

% Initial conditions for system. Bounds if x0 is free s.t. x0l=< x0 <=x0u
% If fixed, x0l == x0u
problem.states.x0l = [1 0]; 
problem.states.x0u = [1 0]; 

% State bounds. xl=< x <=xu
problem.states.xl = [-inf -inf];
problem.states.xu = [+inf +inf];

% Terminal state bounds. xfl=< xf <=xfu. If fixed: xfl == xfu
problem.states.xfl = [-inf -inf]; 
problem.states.xfu = [+inf +inf];

% Input bounds
problem.inputs.ul = [0];
problem.inputs.uu = [1];
%-------------- END CODE ---------------
end


%% System dynamics
function [dx] = dynamics(x,u,t)
k1=1; k2=10; k3=1;
dx1 = -u(1)*(k1*x(1)-k2*x(2));
dx2 =  u(1)*(k1*x(1)-k2*x(2)) - (1-u(1))*k3*x(2);
%dx3 =  u(1)*(k1*x(1)-k2*x(2)) + u(1)*(k1*x(1)-k2*x(2)) + (1-u(1))*k3*x(2);
dx = [dx1; dx2];
end

%% Cost functions

function lag = stageCost(x,u,t)
lag = 0;
end
function mayer = terminalCost(x,u,t)
% mayer = x(3);
mayer = x(1) + x(2) - 1;

end
