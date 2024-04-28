## Formulation of the Multi-agent path planning problem in QuITO v.2 with active mesh refinement
In the problem definition file **AUVpathplan.m**, we first encode the function handles for system dynamics:
```matlab
% Set system dynamics
problem.dynamicsFunc = @dynamics;
```
Function handles for the cost function:  
```matlab
% Set Lagrange cost (Stagewise cost) to be minimized
problem.stageCost = @stageCost;

% Set Mayer cost (Terminal cost)
problem.terminalCost = @terminalCost;
```
Specify the time variables:
```matlab
% Initial time. t0<tf. NOTE: t_0 has to be zero.
problem.time.t0 = 0; 

% Final time. tf is fixed.
problem.time.tf = 10;
```
Specify the state and control dimension:
```matlab
% Number of states.
problem.nx = 18;

% Number of inputs.
problem.nu = 18;
```
For **state variables** we specify initial conditions: 
```matlab
% Initial conditions for system. Bounds if x0 is free s.t. x0l=< x0 <=x0u
% If fixed, x0l == x0u
uav1pos=[10 10 10];uav2pos=[6 10 6];uav3pos=[10 6 10];
problem.states.x0l = [uav1pos 0 0 0 uav2pos 0 0 0 uav3pos 0 0 0]; 
problem.states.x0u = [uav1pos 0 0 0 uav2pos 0 0 0 uav3pos 0 0 0];
```
State bounds:
```matlab
% State bounds. xl=< x <=xu
problem.states.xl = [0 0 0 -pi -pi -pi 0 0 0 -pi -pi -pi 0 0 0 -pi -pi -pi]; % Lower bound on state
problem.states.xu = [15 15 15 pi pi pi 15 15 15 pi pi pi 15 15 15 pi pi pi]; % Upper bound on state
```
Terminal state bounds:
```matlab
% Terminal state bounds. xfl=< xf <=xfu. If fixed: xfl == xfu
uav1final=[0 0 0]; uav2final=[2 0 0];uav3final=[0 2 0];
problem.states.xfl = [uav1final 0 0 0 uav2final 0 0 0 uav3final 0 0 0];
problem.states.xfu = [uav1final 0 0 0 uav2final 0 0 0 uav3final 0 0 0];
```
For **control variables** we specify bounds:
```matlab
% Input bounds
bound=100;
problem.inputs.ul = [-bound -bound -bound -bound -bound -bound -bound -bound -bound -bound -bound -bound -bound -bound -bound -bound -bound -bound]; 
problem.inputs.uu = [bound bound bound bound bound bound bound bound bound bound bound bound bound bound bound bound bound bound]; 
```
Next, we define the system dynamics by specifying the respective ODEs in the function **dynamics**: 
```matlab
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
```
The Lagrange cost is defined in the function **stageCost**:
```matlab
Q=diag([1;1;1;50;50;50;1;1;1;50;50;50;1;1;1;50;50;50]);R=diag([1;10;10;1;5;10;1;10;10;1;5;10;1;10;10;1;5;10]);
lag = x'*Q*x + u'*R*u;
```
The Mayer cost (terminal cost) is defined in the function **terminalCost**:
```matlab
mayer = 0;
```
In order to incorporate mixed constraints, a copy of **src/problemTranscription/solveProblem.m** and **src/problemTranscription/transcriptionSolve.m**  is added in this directory, with the following additional lines in the **transcriptionSolve.m** file , 
```matlab
for i = 1 : num_of_steps + 1
    % Ellipsoidal obtsacle avoidance path constraints
    opti.subject_to((X(1, i)- 8.3)^2 + (X(2, i)- 8.3)^2 + (X(3, i)- 8)^2 >= 1.7^2);
    opti.subject_to((X(1, i)- 3)^2 + (X(2, i)- 6.5)^2 + (X(3, i)- 5)^2 >= 2.7^2);
    opti.subject_to((X(1, i)- 7.3)^2 + (X(2, i)- 2.7)^2 + (X(3, i)- 5)^2 >= 2.7^2);   
    opti.subject_to((X(1, i)- 10)^2 + (X(2, i)- 10)^2 + (X(3, i)- 2)^2 >= 2.7^2);

    opti.subject_to((X(7, i)- 8.3)^2 + (X(8, i)- 8.3)^2 + (X(9, i)- 8)^2 >= 1.7^2);
    opti.subject_to((X(7, i)- 1.7)^2 + (X(8, i)- 1.7)^2 + (X(9, i)- 2)^2 >= 1.7^2);
    opti.subject_to((X(7, i)- 3)^2 + (X(8, i)- 6.5)^2 + (X(9, i)- 5)^2 >= 2.7^2);
    opti.subject_to((X(7, i)- 7.3)^2 + (X(8, i)- 2.7)^2 + (X(9, i)- 5)^2 >= 2.7^2);
    opti.subject_to((X(7, i)- 10)^2 + (X(8, i)- 10)^2 + (X(9, i)- 2)^2 >= 2.7^2);
    
    opti.subject_to((X(13, i)- 8.3)^2 + (X(14, i)- 8.3)^2 + (X(15, i)- 8)^2 >= 1.7^2);
    opti.subject_to((X(13, i)- 3)^2 + (X(14, i)- 6.5)^2 + (X(15, i)- 5)^2 >= 2.7^2);
    opti.subject_to((X(13, i)- 7.3)^2 + (X(14, i)- 2.7)^2 + (X(15, i)- 5)^2 >= 2.7^2);
    opti.subject_to((X(13, i)- 10)^2 + (X(14, i)- 10)^2 + (X(15, i)- 2)^2 >= 2.7^2);

    %Cuboidal obstacle avoidance constraints
    opti.subject_to((X(1, i)- 1.7)^10 + (X(2, i)- 1.7)^10 + (X(3, i)- 2)^10 >= 2^10); 
    opti.subject_to(((X(1, i)- 2)/2)^10 + ((X(2, i)- 8.5)/3)^10 + ((X(3, i)- 5)/5)^10 >= 1); 
    opti.subject_to(((X(1, i)- 14)/2)^10 + ((X(2, i)- 1)/3)^10 + ((X(3, i)- 7.5)/10)^10 >= 1); 
    
    opti.subject_to((X(7, i)- 1.7)^10 + (X(8, i)- 1.7)^10 + (X(9, i)- 2)^10 >= 2^10); 
    opti.subject_to(((X(7, i)- 2)/2)^10 + ((X(8, i)- 8.5)/3)^10 + ((X(9, i)- 5)/5)^10 >= 1);
    opti.subject_to(((X(7, i)- 14)/2)^10 + ((X(8, i)- 1)/3)^10 + ((X(9, i)- 7.5)/10)^10 >= 1); 
    
    opti.subject_to((X(13, i)- 1.7)^10 + (X(14, i)- 1.7)^10 + (X(15, i)- 2)^10 >= 2^10); 
    opti.subject_to(((X(13, i)- 2)/2)^10 + ((X(14, i)- 8.5)/3)^10 + ((X(15, i)- 5)/5)^10 >= 1); 
    opti.subject_to(((X(13, i)- 14)/2)^10 + ((X(14, i)- 1)/3)^10 + ((X(15, i)- 7.5)/10)^10 >= 1); 
    
    % UAV collision avoidance path constraints
    opti.subject_to((X(7, i)-X(1, i) )^2 + (X(8, i)- X(2, i))^2 + (X(9, i)- X(3, i))^2 >= 2);
    opti.subject_to((X(7, i)-X(13, i) )^2 + (X(8, i)- X(14, i))^2 + (X(9, i)- X(15, i))^2 >= 2);
    opti.subject_to((X(13, i)-X(1, i) )^2 + (X(14, i)- X(2, i))^2 + (X(15, i)- X(3, i))^2 >= 2);

end
```
After defining the problem data i.e., the dynamics, the constriants, and the objective we move towards setting up the optimization problem and other parameters in the  **options.m** file. 
We define the default quasi-interpolation parameters (if not passed as input to the **options** function):
```matlab
options.variance = 2; % D = 2 default
generating_function_flag = 1; % default
```
The desired generating function for the approximation is chosen as: 
```matlab
% Select a generating function as per flag
%---------------------------------------
% Gaussian order 2                 (1)
% Laguerre gaussian order 4        (2) 
% Laguerre gaussian order 6        (3) 
% Hermite polynomial order 10      (4)
% Trigonometric guassian order 4   (5)
% Hyperbolic secant order 2        (6) 
options.generating_function=generating_function_flag;
```
The desired discretization scheme is chosen as: 
```matlab
% Euler method              ('euler')
% Trapezoidal method        ('trapezoidal') 
% Hermite-Simpson method    ('hermite') 
% Runge-kutta 4 method      ('RK4')
options.discretization='euler';
```
For solving the optimization problem we choose the interior point solver **IPOPT**:
```matlab
options.NLPsolver = 'ipopt';
```
The associated IPOPT solver settings are:
```matlab
options.ipopt.tol=1e-9;
options.ipopt.print_level=5;
options.ipopt.max_iter=5000;
options.ipopt.mu_strategy ='adaptive';
options.ipopt.hessian_approximation='exact';
options.ipopt.limited_memory_max_history=6;
options.ipopt.limited_memory_max_skipping=1;
```
Meshing strategy is chosen as:
```matlab
options.mesh_strategy='mesh refinement';
```
Mesh refinement settings are chosen as:
```matlab
%% Mesh Refinement settings 
options.MR_termination_tol= 10;
options.MR_width_factor= 21; % Set as a fraction of the time horizon (T)
                             % Refinement width parameter = T / options.MR_width_factor
options.MR_rate=4;           % Gridding rate 
options.MR_max_iter=10;      % Limit on number of refinement iterations 
```
We specify the following output and exit variables:
```matlab
% Display computation time
options.print.time = 1;

% Display cost (objective) values
options.print.cost = 1;
```
The plotting setting is specified as:
```matlab
% 0: Do not plot
% 1: Plot only action trajectory
% 2: Plot all figures (state and input trajectory)
options.plot = 2;
```
The plotting settings to visualize mesh refinement statistics are specified as :
```matlab
% Mesh refinement plot options
% 0: Do not plot
% 1: Plot mesh history
% 2: Plot iterative cost variation
% 3: Plot all above
options.MRplot=3;
```
### Results
Finally, in order to solve the optimization problem and observe the results, we run the main file **main.m**.
We fetch the problem and options and consequently solve the resultant NLP:
```matlab
%% Set-up and solve problem

problem = AUVpathplan;          % Fetch the problem definition
options = options(50, 2);        % Get options and solver settings (N,D),
                               %where step size h=(tf-t0)/N
solution = solveProblem(problem, options);
```
We plot the results by using the **postProcess.m** file:
```matlab
postProcess(solution, problem, opts)
```


