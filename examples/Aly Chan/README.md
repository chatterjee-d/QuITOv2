## Formulation of the Aly Chan problem in QuITO v.2
In problem definition file **AlyChan.m**, we first encode the function handles for system dynamics:
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
problem.time.tf = pi/2;
```
Specify the state and control dimension:
```matlab
% Number of states.
problem.nx = 3;

% Number of inputs.
problem.nu = 1;
```
For **state variables** we specify initial conditions:
```matlab
% Initial conditions for system. Bounds if x0 is free s.t. x0l=< x0 <=x0u
% If fixed, x0l == x0u
problem.states.x0l = [0 1 0]; 
problem.states.x0u = [0 1 0];
```
State bounds:
```matlab
% State bounds. xl=< x <=xu
problem.states.xl = [-inf -inf -inf];
problem.states.xu = [inf inf inf];
```
Terminal state bound:
```matlab
% Terminal state bounds. xfl=< xf <=xfu. If fixed: xfl == xfu
problem.states.xfl = [-inf -inf -inf]; 
problem.states.xfu = [inf inf inf];
```
For **control variables** we specify bounds:
```matlab
% Input bounds
problem.inputs.ul = -1;
problem.inputs.uu = 1;
```
Next, we define the system dynamics by specifying the respective ODEs in the function **dynamics**:
```matlab
dx1 = x(2);
dx2 = u(1);
dx3 = 0.5 * (x(2)^2 - x(1)^2);
```
The Lagrange cost is defined in the function **stageCost**:
```matlab
lag = 0;
```
The Mayer cost (terminal cost) is defined in the function **terminalCost**:
```matlab
mayer = x(3);
```
After defining the problem data i.e., the dynamics, the constriants, and the objective we move towards setting up the optimization problem and other parameters in the **options.m** file. We define the default quasi-interpolation parameters (if not passed as input to the **options** function):
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
For solving the nonlinear optimization problem the **IPOPT** solver is employed:
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
Meshing options is chosen as:
```matlab
options.meshstrategy='fixed';
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
### Results
Finally, in order to solve the optimization problem and observe the results, we run the main file **main.m**. We fetch the problem and options and consequently solve the resultant NLP:
```matlab
%% Set-up and solve problem

problem = AlyChan;          % Fetch the problem definition
opts = options(100, 2);        % Get options and solver settings (N,D),
                               %where step size h=(tf-t0)/N
solution = solveProblem(problem, opts);
```
We plot the results by using the **postProcess.m** file:
```matlab
postProcess(solution, problem, opts)
```


