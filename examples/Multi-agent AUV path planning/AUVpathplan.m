%% Problem formulation
function [problem] = AUVpathplan
% Rayleigh problem with control constraints
%
% Syntax:  [problem] = RayleighProblemControlConstraints
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
problem.nx = 18;

% Number of inputs.
problem.nu = 18;

% Initial conditions for system. Bounds if x0 is free s.t. x0l=< x0 <=x0u
% If fixed, x0l == x0u
uav1pos=[10 10 10];uav2pos=[6 10 6];uav3pos=[10 6 10];
problem.states.x0l = [uav1pos 0 0 0 uav2pos 0 0 0 uav3pos 0 0 0]; % Lower bound on initial state
problem.states.x0u = [uav1pos 0 0 0 uav2pos 0 0 0 uav3pos 0 0 0]; % Upper bound on initial state

% State bounds. xl=< x <=xu
problem.states.xl = [0 0 0 -pi -pi -pi 0 0 0 -pi -pi -pi 0 0 0 -pi -pi -pi]; % Lower bound on state
problem.states.xu = [15 15 15 pi pi pi 15 15 15 pi pi pi 15 15 15 pi pi pi]; % Upper bound on state

% Terminal state bounds. xfl=< xf <=xfu. If fixed: xfl == xfu
uav1final=[0 0 0]; uav2final=[2 0 0];uav3final=[0 2 0];
problem.states.xfl = [uav1final 0 0 0 uav2final 0 0 0 uav3final 0 0 0]; % Lower bound on final state
problem.states.xfu = [uav1final 0 0 0 uav2final 0 0 0 uav3final 0 0 0]; % Upper bound on final state

% Input bounds
bound=100;
problem.inputs.ul = [-bound -bound -bound -bound -bound -bound -bound -bound -bound -bound -bound -bound -bound -bound -bound -bound -bound -bound]; % Lower bound on control
problem.inputs.uu = [bound bound bound bound bound bound bound bound bound bound bound bound bound bound bound bound bound bound]; % Upper bound on control

%-------------- END CODE ---------------
end

function X= hat(x)
X=[0 x(3) -x(2) ; -x(3) 0 x(1) ; x(2) -x(1) 0 ];
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
ws1=[x(1); x(2);x(3)]; wr1=[x(4);x(5);x(6)];taus1=[u(1);u(2);u(3)];taur1=[u(4);u(5);u(6)];
m= 10; Ds=diag([0.1;5;5]);Dr=diag([5;5;0.1]); Icg = [3.45 0 1.28e-15; 0 3.45 0; 1.28e-15 0 2.2];
dws1= (1/m)* ((hat(ws1)*wr1) - Ds*ws1 + taus1);
dwr1 =  Icg\((hat(Icg*wr1)*wr1) - Dr*wr1 + taur1);

ws2=[x(7); x(8);x(9)]; wr2=[x(10);x(11);x(12)];taus2=[u(7);u(8);u(9)];taur2=[u(10);u(11);u(12)];
m= 10; Ds=diag([0.1;5;5]);Dr=diag([5;5;0.1]); Icg = [3.45 0 1.28e-15; 0 3.45 0; 1.28e-15 0 2.2];
dws2= (1/m)* ((hat(ws2)*wr2) - Ds*ws2 + taus2);
dwr2 =  Icg\((hat(Icg*wr2)*wr2) - Dr*wr2 + taur2);

ws3=[x(13); x(14);x(15)]; wr3=[x(16);x(17);x(18)];taus3=[u(13);u(14);u(15)];taur3=[u(16);u(17);u(18)];
m= 10; Ds=diag([0.1;5;5]);Dr=diag([5;5;0.1]); Icg = [3.45 0 1.28e-15; 0 3.45 0; 1.28e-15 0 2.2];
dws3= (1/m)* ((hat(ws3)*wr3) - Ds*ws3 + taus3);
dwr3 =  Icg\((hat(Icg*wr3)*wr3) - Dr*wr3 + taur3);

dx = [dws1; dwr1; dws2; dwr2; dws3; dwr3];

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
% w=[x(1); x(2);x(3);x(4);x(5);x(6)];tau=[u(1);u(2);u(3);u(4);u(5);u(6)];
Q=diag([1;1;1;50;50;50;1;1;1;50;50;50;1;1;1;50;50;50]);R=diag([1;10;10;1;5;10;1;10;10;1;5;10;1;10;10;1;5;10]);
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
