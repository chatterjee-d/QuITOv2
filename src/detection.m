function [Tstr]= detection(solution,problem, options)
    opti=solution.output;
    % Problem related
    t0 = problem.time.t0; % start time; NOTE: t0 has to be zero
    tf = problem.time.tf;  % end time
    num_of_steps = options.nodes; % should be even
    
    sampling_N=128;
    sampling_time= linspace(t0,tf,sampling_N);
    signal=[];
    for i = 1 : sampling_N
        signal=[signal,opti.value(solution.U_hat(sampling_time(i)))];
    end
    
%% Define wavelet parameters
    fs=0.001;
    wname =  'mexh'; %'haar'; % Haar wavelet
    scales = 0.0001:0.0001:fs;  % Scales for the transform
    
    detect=[];
    
    for j=1:problem.nu
        signal_lst=signal(j,:);
        % Compute the continuous wavelet transform
        coeffs = cwt(signal_lst,scales,wname);
    


        [peaks,Tstr] =findpeaks(abs(coeffs(1,:)));
        Tstr= sampling_time(Tstr);
        % Find local maxima and their properties
        [peaks, locs, ~, proms] = findpeaks(abs(coeffs(1,:)));

        [~, sorted_idx] = sort(proms, 'descend');

        Tstr =  sampling_time(locs(sorted_idx));
        detect = [detect,Tstr(1:2)];
    end
    Tstr=detect;



    
%     
    
end
