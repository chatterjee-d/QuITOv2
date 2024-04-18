function [U_out,U_app]= postProcess(solution, problem, options)
% postProcess - Show all the the required plots and process the solution
%
% Syntax:  postProcess(solution, problem, options)
%
% Output:
%    returns none

%------------- BEGIN CODE --------------
if strcmp(options.mesh_strategy,'fixed')
    quito_solution= solution;
elseif strcmp(options.mesh_strategy,'mesh refinement')
    quito_solution= solution.ref_solution;
end
opti = quito_solution.output;
t0=problem.time.t0;
tf=problem.time.tf;
tau=quito_solution.tau;
num_of_steps = length(tau)-1;
X = quito_solution.X;

if options.print.time
    disp(newline+"Time taken to solve: "+num2str(solution.computation_time)+" secs")
end

if options.NLPsolver == "ipopt"
    disp("Total iterations performed by NLP solver is "+num2str(quito_solution.output.stats.iter_count))
end

if options.print.cost
    disp(newline+"The cost is: "+num2str(quito_solution.cost))
end

% Extract optimal values
X_out = zeros(problem.nx, num_of_steps + 1);
U_out = zeros(problem.nu, num_of_steps);
for i = 1 : num_of_steps + 1
    X_out(:, i) = opti.value(X(:, i));
end
for i = 1 : num_of_steps
    U_out(:, i) =   opti.value(quito_solution.U_hat(tau(i))); %U_app(:, i);
end

% Figure
time_horizon = linspace(problem.time.t0, problem.time.tf, num_of_steps+1);

if options.plot
    % Plot action trajectories
    for ui = 1 : problem.nu
        figure
        hold on;
        plot(tau(1:end-1), U_out(ui,:),'b','LineWidth',1);
        ylabel("Action trajectory");
        xlabel('Time [s]');
        hold off;
        xlim([problem.time.t0, problem.time.tf])
        grid on
        label= sprintf('$u_{%d}$',ui);
        legend(label,'Interpreter','Latex');
%         legend("u"+num2str(ui));
    end
    
    % Plot state trajectories
    if options.plot == 2
        for xi = 1 : problem.nx
            figure
            plot(tau(1:end), X_out(xi,:),'b','LineWidth',1);
            ylabel("State trajectory");
            xlabel('Time [s]');
            xlim([problem.time.t0, problem.time.tf])
            grid on
            label= sprintf('$x_{%d}$',xi);
            legend(label,'Interpreter','Latex');
        end
    end
end

maroon= [0.6350 0.0780 0.1840];
if strcmp(options.mesh_strategy,'mesh refinement') && options.MRplot==1
    figure;

    iter= length(solution.cost_arr)-1;
    hold on;
    grid on;
    plot(solution.solution_knot.tau, 1,'o','Color', maroon,'MarkerSize',7);
    for i = 1:iter
        plot(solution.tau_history{i}, i+1,'bo','MarkerSize',7);
    end
    ylabel('Iteration');
    yticks(1:1:iter+1);
    xlabel('Time');
    title('Mesh refinement history');
    hold off;
elseif strcmp(options.mesh_strategy,'mesh refinement') && options.MRplot==2
    iter= length(solution.cost_arr)-1;
    figure;
    plot(1:length(solution.cost_arr(1:end)),solution.cost_arr,'b-s');
    xlabel('Iteration');
    ylabel("Cost");
    xticks(1:1:iter+1);
    grid on;
    ax = gca ;
    ax.GridAlpha = 0.5; 
    ax.GridColor = maroon;  
elseif strcmp(options.mesh_strategy,'mesh refinement') && options.MRplot==3
    iter= length(solution.cost_arr)-1;
    figure;
    hold on;
    grid on;
    plot(solution.solution_knot.tau, 1,'o','Color', maroon,'MarkerSize',7);
    for i = 1:iter
        plot(solution.tau_history{i}, i+1,'bo','MarkerSize',7);
    end
    ylabel('Iteration');
    yticks(1:1:iter+1);
    xlabel('Time');
    title('Mesh refinement history');
    hold off;
    
    figure;
    plot(1:length(solution.cost_arr(1:end)),solution.cost_arr,'b-s');
    xlabel('Iteration');
    ylabel("Cost");
    xticks(1:1:iter+1);
    grid on;
    ax = gca ;
    ax.GridAlpha = 0.5; 
    ax.GridColor = maroon;
end



%-------------- END CODE ---------------
end