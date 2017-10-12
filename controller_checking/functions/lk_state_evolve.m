function [x] = lk_state_evolve( varargin )
%	let_s_evolve()
%		The objective of this function is to calculate how x will evolve as
%		t increases in the lane keeping system and starting at initial state x0.
%
%		Future Work:
%			- Add support for discrete version of LK system. 
%
%		Example Usage:
%			- let_s_evolve( x0 , u_traj , mu_traj , r_d_traj , con )
%			- let_s_evolve( ... , 'Model',name)
%
%
%		Inputs:
%			- sys :	A struct containing system matrices and information.
%					Should contain: .B,.E,.x0,.Ts

%% Input Processing

if nargin < 6
	error('Not enough input arguments.')
end

%Save all necessary variables

x0 		= varargin{1};
u_traj 	= varargin{2};
mu_traj = varargin{3};
r_d_traj = varargin{4};
LK_cont	= varargin{5}; 		%Necessary constants for calculating the system matrices
Ts 		= varargin{6};

% Calculate T
T1 = size(u_traj,1);
T2 = size(mu_traj,1);

if T1 ~= T2
	error('The sizes of T1 and T2 are incompatible.')
end

T = T1;

%Check if we want to select a different model.
model_name = 'Petter'; %Default model

if any( strcmp(varargin,'Model') )
	model_name = varargin{find(strcmp(varargin,'Model'))+1};
	if ~any(strcmp({'Petter','Shaobing'},model_name))
		error('Unrecognized Model name.')
	end
end

%Checking fields of sys struct

% con_fields = {'Caf','Car'};
% for n = 1 : length(sys_fields)

% 	if ~any(isfield(sys,sys_fields{n}))
% 		error([ 'Missing ' sys_fields{n} ' field.' ] );
% 	end

% end

%% Constants

%% Calculation of Trajectory.
x = x0;
for t = 1 : T

	% Get current speed.
	mu = mu_traj(t);

	%-----------------------
	% Create System Matrices
	%-----------------------
	if strcmp(model_name,'Petter')
		A_lk = [0, 1, mu, 0; 
	          0, -(LK_cont.Caf+LK_cont.Car)/LK_cont.M/mu, 0, ((LK_cont.lr*LK_cont.Car-LK_cont.lf*LK_cont.Caf)/LK_cont.M/mu - mu); 
	          0, 0, 0, 1;
	          0, (LK_cont.lr*LK_cont.Car-LK_cont.lf*LK_cont.Caf)/LK_cont.Iz/mu,  0, -(LK_cont.lf^2 * LK_cont.Caf + LK_cont.lr^2 * LK_cont.Car)/LK_cont.Iz/mu];
	else
		A_lk = [0, 1, 0, 0; 
	        	0, -(LK_cont.Caf+LK_cont.Car)/LK_cont.M/mu, 0, ((LK_cont.lr*LK_cont.Car-LK_cont.lf*LK_cont.Caf)/LK_cont.M/mu - mu); 
	          	0, 0, 0, 1;
	          	0, (LK_cont.lr*LK_cont.Car-LK_cont.lf*LK_cont.Caf)/LK_cont.Iz/mu,  0, -(LK_cont.lf^2 * LK_cont.Caf + LK_cont.lr^2 * LK_cont.Car)/LK_cont.Iz/mu];
	end

    B_lk = [0; LK_cont.Caf/LK_cont.M; 0; LK_cont.lf*LK_cont.Caf/LK_cont.Iz];
	E_lk = [0; 0; -1; 0];

	%-------------
	% State update
	%-------------

	lk_state_dot =  A_lk*x(:,end) + B_lk * u_traj(t) + E_lk* r_d_traj(t); 
	x = [ x x(:,end)+lk_state_dot*Ts ];
	
end