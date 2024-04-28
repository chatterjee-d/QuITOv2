## Formulation of the SIRI in QuITO v.2 with active mesh refinement
In the problem definition file **siri.m**, we first encode the function handles for system dynamics:
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
problem.time.tf = 20;
```
Specify the state and control dimension:
```matlab
% Number of states.
problem.nx = 3;

% Number of inputs.
problem.nu = 3;
```
For **state variables** we specify initial conditions: 
```matlab
% Initial conditions for system. Bounds if x0 is free s.t. x0l=< x0 <=x0u
% If fixed, x0l == x0u
problem.states.x0l = [0.2 0.8 0]; 
problem.states.x0u = [0.2 0.8 0]; 
```
State bounds:
```matlab
% State bounds. xl=< x <=xu
problem.states.xl =  [-inf 0 0]; % Lower bound on state
problem.states.xu =  [+inf 1 1]; % Upper bound on state
```
Terminal state bounds:
```matlab
% Terminal state bounds. xfl=< xf <=xfu. If fixed: xfl == xfu
problem.states.xfl = [-inf 0 0]; % Lower bound on final state
problem.states.xfu = [+inf 1 1]; % Upper bound on final state
```
For **control variables** we specify bounds:
```matlab
% Input bounds
problem.inputs.ul = [0.1 0.1 0]; % Lower bound on control
problem.inputs.uu = [1 1 0.9]; % Upper bound on control
```
Next, we define the system dynamics by specifying the respective ODEs in the function **dynamics**: 
```matlab
cs = 2; cr = 2; ci = 5; cv = 3; beta = 0.7; beta_hat = 0.5; gamma = 0.5; % 0.5 works
dx1 = cs * x(2) + cr * x(3) + ci * ( 1 - x(2) - x(3) ) - cs * x(2) * u(1) - cr * x(3) * u(2) + cv * x(2) * u(3);
dx2 = - beta * x(2) * ( 1 - x(2) - x(3) ) * u(1) - x(2) * u(3);
dx3 = gamma * ( 1 - x(2) - x(3) ) - beta_hat * x(3) * ( 1- x(2) - x(3) ) * u(2) + x(2) * u(3);
dx = [dx1; dx2; dx3];
```
The Lagrange cost is defined in the function **stageCost**:
```matlab
lag = 0;
```
The Mayer cost (terminal cost) is defined in the function **terminalCost**:
```matlab
mayer = x(1);
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
options.MR_termination_tol= 0.001;
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

problem = bressan;          % Fetch the problem definition
options = options(100, 2);        % Get options and solver settings (N,D),
                               %where step size h=(tf-t0)/N
solution = solveProblem(problem, options);
```
We plot the results by using the **postProcess.m** file:
```matlab
postProcess(solution, problem, opts)
```


