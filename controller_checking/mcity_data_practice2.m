function [results] = mcity_data_practice2( varargin )
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
LK.H_u = 100;   % weight in QP for steering (larger -> less aggressive centering)
LK.setup(struct());

for k = t0 : t0+test_duration-1

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

figure;
subplot(2,1,1)
hold on;
plot(t,delta_f)
plot(t,steering_act/ST_RATIO)

xlabel('Time (s)')
ylabel('Steering Angle (rad)')
title('lk\_pcis\_controller Output Vs. Car''s Actual Steering')

legend('LK PCIS Controller','Steering Actual')

subplot(2,1,2)
hold on;
plot(t,delta_f)
plot(t,steering_cmd/ST_RATIO)

xlabel('Time (s)')
ylabel('Steering Angle (rad)')
title('lk\_pcis\_controller Output Vs. Car''s Commanded Steering')

legend('LK PCIS Controller','Steering Command')

%% Create an Input Trajectory that considers the last input

% LK2 = lk_pcis_controller;
% LK2.H_u = 100;
% LK2.f_u = 0;
% LK2.setup(struct());

for k = t0 : t0+test_duration-1

	if k > t0
		%release(LK2);
		% clear LK2;
		% LK = lk_pcis_controller;
		LK.H_u = 100;
		LK.f_u = -LK.H_u*delta_f2(k-1);
		% LK.setup(struct());
	end

	temp_lk_acc_state.y 	= lk_acc_state.y.Data(k);
	temp_lk_acc_state.nu 	= lk_acc_state.nu.Data(k);
	temp_lk_acc_state.dy 	= lk_acc_state.dy.Data(k);
    temp_lk_acc_state.mu 	= lk_acc_state.mu.Data(k);
    temp_lk_acc_state.dPsi 	= lk_acc_state.dPsi.Data(k);
    temp_lk_acc_state.r 	= lk_acc_state.r.Data(k);
    temp_lk_acc_state.h 	= lk_acc_state.h.Data(k);
    temp_lk_acc_state.r_d 	= lk_acc_state.r_d.Data(k);

    %Input for the LK and ACC Systems
	[ delta_f2(k) lk_info2(k) ] = LK.step( temp_lk_acc_state );

end

figure;
hold on;
plot(t,delta_f)
plot(t,delta_f2)

xlabel('Time (s)')
ylabel('Control Input, \delta_f')
title('Trajectories for control inputs when a history-respecting term is added')
legend('Current','Smoothing with u[1]')

%%%%%%%%%%%%%%%%%%%%
%% Saving Results %%
%%%%%%%%%%%%%%%%%%%%

results.name = 'mcity_data_practice2';
results.delta_f = delta_f;
results.lk_info = lk_info;
results.delta_f2 = delta_f2;
results.lk_info2 = lk_info2;
results.t = t;
results.t0 = t0;

end