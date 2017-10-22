% mpcqpsolver_test1.m
%	Testing the ability to offer certain forms of QP to mpcqpsolver

clear all;
close all;
clc;

%%%%%%%%%%%%%%%
%% Constants %%
%%%%%%%%%%%%%%%

u_bar = 12;

H_u = 2;
f_u = -H_u*u_bar;

sol_opts = struct('DataType', 'double', 'MaxIter', 200, ...
                      'FeasibilityTol', 1e-6, 'IntegrityChecks', true);

%%%%%%%%%%%%%%%%%%%%%
%% Problem Solving %%
%%%%%%%%%%%%%%%%%%%%%

[u,status] = mpcqpsolver(	chol(H_u,'lower')\eye(size(H_u)), ...
							f_u, ...
							1,0,...
							[],zeros(0,1),...
							false(size(0,1)),sol_opts)

[u2,fval,sol_codes] = quadprog( H_u , f_u , ...
								-1,0 )