% Script to solve the Optimal Control Problem. Run this file.
% Formulate the problem and settings in the other two function files.

% -------------------------------------------------------------------------
% Primary Contributors: 
% - Rihan Aaron D'Silva, Indian Institute of Technology Bombay
% - Siddhartha Ganguly, Indian Institute of Technology Bombay
% Refer the article: S. Ganguly, N. Randad, D. Chatterjee, and R. Banavar
% Constrained trajectory synthesis via quasi-interpolation, 
% IEEE Conference on Decision & Control, 2022, Cancun, Mexico
% -------------------------------------------------------------------------
clear all;
close all;

%% Set-up and solve problem

problem = TemplateProblem;          % Fetch the problem definition
opts = options(100, 2);        % Get options and solver settings (N,D),  
                               %where step size h=(tf-t0)/N
solution = solveProblem(problem, opts);

%% Post-processing

postProcess(solution, problem, opts)
