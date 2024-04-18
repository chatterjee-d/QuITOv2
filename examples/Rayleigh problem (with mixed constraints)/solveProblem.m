function [solution] = solveProblem(problem, options)
tic()

% Problem related
t0 = problem.time.t0;  % start time; NOTE: t0 has to be zero
tf = problem.time.tf;  % end time
num_of_steps = options.nodes; % should be even
tau_knot=linspace(t0, tf, num_of_steps+1 );
h = (tf - t0) / num_of_steps;

if strcmp(options.mesh_strategy,'fixed')
    solution= transcriptionSolve(tau_knot,problem,options);
elseif strcmp(options.mesh_strategy,'mesh refinement')
    % obtain solution on uniform grid
    solution_knot=transcriptionSolve(tau_knot,problem,options); 
    disp('--------------------------------------------------------------------')
    disp('SOLVED PROBLEM ON UNIFORM MESH');
    % obtain refinement intervals
    % need to incorporate some kind of checks for for poor detection 
    detect= detection(solution_knot,problem, options);
    disp('--------------------------------------------------------------------')
    disp('Obtained refinment intervals');
    % return mesh refinement solution
    solution= mesh_refinement(problem,options,solution_knot,detect);
else
    error('Choose appropriate mesh strategy in options.m file');
end
solution.computation_time=toc();
end