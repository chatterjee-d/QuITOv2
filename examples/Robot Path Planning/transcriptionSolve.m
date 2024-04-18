function [solution] = transcriptionSolve(tau,problem, options)
% solveProblem - main file for solving NLPs
%
% Syntax:  [solution] = solveProblem(problem, options)

%------------- BEGIN CODE --------------

%% Initialization
% Problem related
t0 = problem.time.t0;  % start time; NOTE: t0 has to be zero
tf = problem.time.tf;  % end time

num_of_steps = length(tau)-1; % should be even

% Step-size
h = (tf - t0) /options.nodes;

% State vector
x = casadi.SX.sym('x', problem.nx);

% Control vector
u = casadi.SX.sym('u', problem.nu);

% Time variable
t = casadi.SX.sym('t');

% Model equations
dx = problem.dynamicsFunc(x,u,t);

% Objective term
L = problem.stageCost(x,u,t);

% Continuous time dynamics
dyn_func = casadi.Function( 'dyn_func', {x, u, t}, {dx}, {'x', 'u', 't'}, {'xdot'});
lag_cost = casadi.Function( 'lag_cost', {x, u, t}, {L}, {'x', 'u', 't'}, {'Stage cost'});

%% Set up solver and declare variables
% Start with an empty NLP
opti = casadi.Opti();

% All states and control variables (inclusive of redundant ones)
X = opti.variable(problem.nx, length(tau));
% X.set_initial(zeros(problem.nx, len(tau)))

U = opti.variable(problem.nu, length(tau));
% U.set_initial(zeros(problem.nu, len(tau)))

%! Check the size of U (no control required at tf)

%% Formulate NLP

% Quasi-interpolation
D = options.variance;
U_hat = @(t) M_hd_x(t, tau, U, h, D, options);

% Lagrange cost array and store U_hat at nodes and cost function quadrature
U_app = opti.variable(problem.nu, length(tau));
for i = 1 : num_of_steps + 1
    U_app(:, i) = U_hat(tau(i));
end

J = 0;
for i = 1 : num_of_steps
    step_size= tau(i+1)-tau(i);
    % Euler rule
    if strcmp(options.discretization,'euler')
        J = J + lag_cost(X(:,i), U_app(:,i), tau(i)) * step_size;
    end
    % Trapazoidal quadrature
    if strcmp(options.discretization,'trapezoidal')
        inte = (lag_cost(X(:,i), U_app(:,i), tau(i)) + lag_cost(X(:,i+1), U_app(:,i+1), tau(i+1)))/2;
        J = J + inte * step_size;
    end
    % Hermite-Simpson's rule
    if strcmp(options.discretization,'hermite')
        if mod(i,2) == 1
            inte = (lag_cost(X(:,i), U_app(:,i), tau(i)) + 4 * lag_cost(X(:,i+1), U_app(:,i+1), tau(i+1)) + lag_cost(X(:,i+2), U_app(:,i+2), tau(i+2)))/6;
            J = J + inte * (2 * h);
        end
    end
    %Runge-Kutta 4 integration
    if strcmp(options.discretization,'RK4')
        k1 = lag_cost(X(:,i),        U_app(:,i), tau(i));
        k2 = lag_cost(X(:,i)+h/2*k1, U_app(:,i), tau(i)+h/2);
        k3 = lag_cost(X(:,i)+h/2*k2, U_app(:,i), tau(i)+h/2);
        k4 = lag_cost(X(:,i)+h*k3,   U_app(:,i), tau(i)+h);
        J = J + h/6*(k1+3*k2+2*k3+k4);
    end
end

% Mayer term
J = J + problem.terminalCost(X(:,end), U_app(:,end), tau(end));

%% Numerical integration and constraint to make zero gap
for k = 1 : num_of_steps % loop over control intervals
   step_size= tau(k+1)-tau(k);
   % Runge-Kutta 4 integration
   k1 = dyn_func(X(:,k),        U_app(:,k), tau(k));
   k2 = dyn_func(X(:,k)+step_size/2*k1, U_app(:,k), tau(k)+step_size/2);
   k3 = dyn_func(X(:,k)+step_size/2*k2, U_app(:,k), tau(k)+step_size/2);
   k4 = dyn_func(X(:,k)+step_size*k3,   U_app(:,k), tau(k)+step_size);
   x_next = X(:,k) + step_size/6*(k1+2*k2+2*k3+k4);
   opti.subject_to(X(:,k+1) == x_next); % close the gaps
end

%% Constraints
% State boundary constraints
for xid = 1 : problem.nx
    % Initial conditions
    if problem.states.x0l(xid) == problem.states.x0u(xid)
        opti.subject_to(X(xid, 1) == problem.states.x0l(xid))
    else
        opti.subject_to(X(xid, 1) >= problem.states.x0l(xid));
        opti.subject_to(X(xid, 1) <= problem.states.x0u(xid));
    end
    % Final conditions
    if problem.states.xfl(xid) == problem.states.xfu(xid)
        opti.subject_to(X(xid, end) == problem.states.xfl(xid))
    else
        opti.subject_to(X(xid, end) >= problem.states.xfl(xid));
        opti.subject_to(X(xid, end) <= problem.states.xfu(xid));
    end
end

% State bounds
for xid = 1 : problem.nx
    for i = 1 : num_of_steps + 1
        opti.subject_to(X(xid, i) >= problem.states.xl(xid));
        opti.subject_to(X(xid, i) <= problem.states.xu(xid));
    end
end

% Control action bounds
for uid = 1 : problem.nu
    for i = 1 : num_of_steps + 1
        opti.subject_to(U_app(uid, i) >= problem.inputs.ul(uid));
        opti.subject_to(U_app(uid, i) <= problem.inputs.uu(uid));
    end
end

% Nonlinear path constraints
a1 = 40; b1 = 20;
a2 = 55; b2 = 40;
a3 = 45; b3 = 65;
r1 = 10; r2 = 80;
for i = 1 : num_of_steps + 1
  opti.subject_to((X(1,i)-a1)^2 + (X(2,i)-b1)^2 >= r1^2);
  opti.subject_to((X(1,i)-a1)^2 + (X(2,i)-b1)^2 <= r2^2);
  opti.subject_to((X(1,i)-a2)^2 + (X(2,i)-b2)^2 >= r1^2);
  opti.subject_to((X(1,i)-a2)^2 + (X(2,i)-b2)^2 <= r2^2);
  opti.subject_to((X(1,i)-a3)^2 + (X(2,i)-b3)^2 >= r1^2);
  opti.subject_to((X(1,i)-a3)^2 + (X(2,i)-b3)^2 <= r2^2);
end

%% Optimization solver
opti.minimize(J)  % minimise the objective function

% NLP solver used here is ipopt
opts = struct('ipopt',options.ipopt);
opti.solver(options.NLPsolver, opts)  % backend NLP solver

disp('Starting to solve...')
tic();
sol = opti.solve();  % Solve the actual problem
solution.NLP_time_taken = toc();
disp('Solving finished!')

% U_out = zeros(problem.nu, num_of_steps);
% for i = 1 : num_of_steps
%     U_out(:, i) =   opti.value(U_hat(tau(i))); %U_app(:, i);
% end
% X_out = zeros(problem.nx, num_of_steps + 1);
% for i = 1 : num_of_steps + 1
%     X_out(:, i) = opti.value(X(:, i));
% end

%% Store to solution
solution.output = sol; % Store CasADi output
solution.X = X;
% solution.U_app = U_app;
solution.tau=value(tau);
solution.cost = opti.value(J);
solution.U_hat=U_hat;
% solution.U=opti.value(U);
% solution.U_out=U_out;
% solution.X_out=X_out;

%-------------- END CODE ---------------
end

% Function approximations
function out = phi(t, tau_i, h, D,options)
%     app-app basis function
% 
%     Args:
%         t: time
%         tau_i: m_i*h
    arg= (t-tau_i)/(h*sqrt(D));
    
    % Mesh refinement case check - only guassian generating functions
    % supported
    if strcmp(options.mesh_strategy,'mesh refinement') && (options.generating_function~=1)
        error('Only guassian generating function supported for mesh refinement');      
    end
    if options.generating_function==1
        % Gaussian order 2
        out = (1 / (sqrt(pi * D))) * exp(-arg^2); 
    elseif options.generating_function==2
        % lagguere gaussian order 4
        out =  (1 / (sqrt(pi * D))) * exp(-arg^2)* (1.5-(arg^2)); 
    elseif options.generating_function==3
        % lagguere gaussian order 6
        out =  (0.5 / (sqrt(pi * D))) * exp(-arg^2)* ((15/4)-(5*arg^2)+ (0.5*arg^4)); 
    elseif options.generating_function==4
        % Hermite polynomial order 10
        out =  (1 / (sqrt(pi * D))) * exp(-arg^2)* ((315/128)-((105/16)*arg^2)+((63/16)*arg^4)-((3/4)*arg^6)+(arg^8/24));
    elseif options.generating_function==5
        % Trigonometric guassian order 4
        out =  (sqrt(exp(1)) / (sqrt(pi * D))) * exp(-arg^2)* cos(sqrt(2)*arg);  
    elseif options.generating_function==6
        % hyperbolic secant
        out = (1/pi*sqrt(D))* sech(arg);
    end

end

function sum = M_hd_x(t, tau, collo_x, h, D, options)
    sum = 0;
    for i = 1:length(tau)
        tau_i = tau(i);
        if i==length(tau)                            %((i== length(tau)) || (i==1))
            h_local=h;           
        else
            h_local= tau(i+1)-tau(i); 
        end

        sum = sum + collo_x(:,i) * phi(t, tau_i, h_local, D, options);
    end
end
