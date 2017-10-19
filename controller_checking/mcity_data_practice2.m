function [] = mcity_data_practice2( varargin )
% 	mcity_data_practice2.m
%		Testing if there is a better way to generate inputs to the LK system.

clear all;
close all;
clc;

%% Constants

ST_RATIO = 16;        % steering ratio of car

t0 = 1;

%Example Dataset
load('data/run-successful.mat');
%load('data/run-snake1.mat');

%Import functions
if ~any(strcmp(path,'../systems/'))
	addpath('../systems/')
	addpath('functions/')
end

%% Collect Data + Create Original Input Trajectory

t 			 = steering_report.SteeringWheelAngle.Time([t0:end]);
steering_cmd = reshape( steering_report.SteeringWheelAngleCommand.Data([t0:end]) , length(t) , 1 );
steering_act = reshape( steering_report.SteeringWheelAngle.Data([t0:end]) , length(t) , 1 );

test_duration = length(t);

x_meas = [ 	reshape(lk_acc_state.y.Data([t0:end]),1,test_duration) ;
			reshape(lk_acc_state.nu.Data([t0:end]),1,test_duration) ;
			reshape(lk_acc_state.dPsi.Data([t0:end]),1,test_duration) ;
			reshape(lk_acc_state.r.Data([t0:end]),1,test_duration) ];

figure;
hold on;
plot(t,steering_act)
plot(t,steering_cmd)

legend('Act','Cmd')

%Collect Data into state trajectories

LK = lk_pcis_controller;
LK.H_u = 4;   % weight in QP for steering (larger -> less aggressive centering)
LK.setup(struct());

for i = t0:t0+length(t)-1
	%Apply LK PCIS controller to data at x

	for k = 1 : test_duration

	temp_lk_acc_state.y 	= lk_acc_state.y.Data(k);
	temp_lk_acc_state.nu 	= lk_acc_state.nu.Data(k);
	temp_lk_acc_state.dy 	= lk_acc_state.dy.Data(k);
    temp_lk_acc_state.mu 	= lk_acc_state.mu.Data(k);
    temp_lk_acc_state.dPsi 	= lk_acc_state.dPsi.Data(k);
    temp_lk_acc_state.r 	= lk_acc_state.r.Data(k);
    temp_lk_acc_state.h 	= lk_acc_state.h.Data(k);
    temp_lk_acc_state.r_d 	= lk_acc_state.r_d.Data(k);

    %Input for the LK and ACC Systems
	[ delta_f(k) lk_info(k) ] = LK.step( temp_lk_acc_state );
	
end

end
