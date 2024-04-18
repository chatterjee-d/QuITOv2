% Script to solve the Optimal Control Problem. Run this file.
% Formulate the problem and settings in the other two function files.

% -------------------------------------------------------------------------
% Primary Contributors: 
% - Rihan Aaron D'Silva, Indian Institute of Technology Bombay
% - Siddhartha Ganguly, Indian Institute of Technology Bombay
% Refer the articles: 
% 1)S. Ganguly, N. Randad, D. Chatterjee, and R. Banavar
% Constrained trajectory synthesis via quasi-interpolation, 
% IEEE Conference on Decision & Control, 2022, Cancun, Mexico
% 2)S. Ganguly, N. Randad, R. A. D’Silva, M. Raj, and D. Chatterjee
% QuITO: Numerical software for constrained nonlinear optimal control problems, 
% SoftwareX, vol. 24, p. 101557, 2023
% -------------------------------------------------------------------------
clear all;close all;

%% Set-up and solve problem

problem = VanDerPol;           % Fetch the problem definition
options = options(100, 2);      % Get options and solver settings (N,D),
                               %where step size h=(tf-t0)/N
solution = solveProblem(problem, options);

%% Post-processing

postProcess(solution, problem, options);
