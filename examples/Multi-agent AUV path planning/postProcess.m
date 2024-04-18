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


% Animation plot
orange = [0.8500 0.3250 0.0980];
yellow= [0.9290 0.6940 0.1250];
purple = [0.4940 0.1840 0.5560];
olive_green = [0.4660 0.8740 0.1880];
maroon= [0.6350 0.0780 0.1840];
gray = [.7 .7 .7];
bad_blue=[0 0.4470 0.7410];
melon='#EDB120';
figure; 
hold on;
view(3);
grid on;
ellipsoid(8.3,8.3,8,1.0,1.0,1.0);
ellipsoid(3,6.5,5,2.0,2.0,2.0);
ellipsoid(7.3,2.7,5,2.0,2.0,2.0);
ellipsoid(10,10,2,2.0,2.0,2.0);
DrawCuboid(2*[1;1;1],[1.7;1.7;2],[0;0;0],olive_green,0.9);
DrawCuboid(2*[1.3;2.3;4.3],[2;14;5],[0;0;0],olive_green,0.9);
DrawCuboid(2*[1.3;2.3;5],[14;1;5],[0;0;0],olive_green,0.9);

plot3(10,10,10,'ro','MarkerSize', 7,MarkerFaceColor = 'r');
plot3(6,10,6,'bo','MarkerSize', 7,MarkerFaceColor = 'b');
plot3(10,6,10,'o','Color',melon,'MarkerSize', 7,MarkerFaceColor = melon);

plot3(0,0,0,'diamond','Color','r','MarkerSize', 7,MarkerFaceColor = 'r');
plot3(2,0,0,'diamond','Color','b','MarkerSize', 7,MarkerFaceColor = 'b');
plot3(0,2,0,'diamond','Color',melon,'MarkerSize', 7,MarkerFaceColor = melon);

for i = 1:size(X_out, 2)
    plot3(X_out(1, 1:i), X_out(2, 1:i), X_out(3, 1:i), 'r-o', 'MarkerSize', 10);
    [xc,yc,zc]=tubeplot([X_out(1,1:i);X_out(2,1:i);X_out(3,1:i)],0.5);
    surf(xc,yc,zc,'FaceColor', 'r', 'EdgeColor', 'none', 'FaceAlpha', 0.1);
    
    plot3(X_out(7, 1:i), X_out(8, 1:i), X_out(9, 1:i), 'b-o', 'MarkerSize', 10);
    [xc,yc,zc]=tubeplot([X_out(7,1:i);X_out(8,1:i);X_out(9,1:i)],0.5);
    surf(xc,yc,zc,'FaceColor', 'b', 'EdgeColor', 'none', 'FaceAlpha', 0.1);
    
    plot3(X_out(13, 1:i), X_out(14, 1:i), X_out(15, 1:i), '-o', 'MarkerSize', 10,'Color',melon);
    [xc,yc,zc]=tubeplot([X_out(13,1:i);X_out(14,1:i);X_out(15,1:i)],0.5);
    surf(xc,yc,zc,'FaceColor', melon, 'EdgeColor', 'none', 'FaceAlpha', 0.1);
    
    drawnow;
    pause(0.15); 
end

%-------------- END CODE ---------------
end