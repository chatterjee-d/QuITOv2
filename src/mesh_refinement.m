function [refine_sol]= mesh_refinement(problem, options,solution_knot,detect)
    t0 = problem.time.t0;  
    tf = problem.time.tf; 
    num_of_steps = options.nodes;
    h = (tf - t0) / num_of_steps;
    tau_knot=linspace(t0, tf, num_of_steps+1 );
    wind= (tf-t0)/options.MR_width_factor;
    Tstr=[];
    for i=1:length(detect)
        arr= detect(i)-wind/2: h/2:detect(i)+wind/2;    
        Tstr=[Tstr,arr];
    end
    
    Qp=solution_knot.cost;
    eps_knot= options.MR_termination_tol;
    max_iter=options.MR_max_iter;
    
    cost_arr=[solution_knot.cost];
    tau_length=[length(tau_knot)];
    tau_history = cell(max_iter, 1);

    for i = 1:max_iter
        iter_count= i; 
        tau=refine(tau_knot,Tstr,iter_count,options);
        ref_solution=transcriptionSolve(tau,problem,options);
        eps=Qp-ref_solution.cost;
        Qp=ref_solution.cost;    
        cost_arr=[cost_arr,ref_solution.cost];
        tau_length=[tau_length,length(tau)];
        tau_history{i} = tau;
        disp("Refinement iteration :" + num2str(iter_count)+ " ; Minimized cost : "+num2str(ref_solution.cost)+ "  ; Cost decrease : "+num2str(eps));
        if eps<0
            disp('Warning! Increase in cost detected on refinement. Now terminating.');
            break
        end    
        if abs(eps)<eps_knot
            disp('error threshold satisfied --> exiting mesh refinement');
            break
        end
    end
    refine_sol.solution_knot=solution_knot;
    refine_sol.ref_solution=ref_solution;
    refine_sol.compute_time= toc();
    refine_sol.cost_arr=cost_arr;
    refine_sol.tau_size_arr=tau_length;
    refine_sol.tau_history=tau_history;
end



function tau = refine(tau_knot,T_str,iter_count,options)
    tau=[];
    for i = 1 : length(tau_knot)-1
        tau=[tau,tau_knot(i)];
        for j=1 : length(T_str)
            if (T_str(j)<=tau_knot(i+1)) && (T_str(j)>=tau_knot(i))
                lst=   linspace(tau_knot(i),tau_knot(i+1),2+iter_count*options.MR_rate);
                tau=[tau,lst(2:end-1)];
                break
            end
        end

    end  
    tau=[tau,tau_knot(end)];
 end