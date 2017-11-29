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

disp('mcity_data_practice2.m')
disp('This set of experiments will:')
disp('1. Replicate the Steering Commands given to the car using the lk_pcis_controller')
disp('2. Implement a version of lk_pcis_controller that weights not just the input')
disp('   magnitude, but the difference between the selected input and the previous input')

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
plot(t,steering_cmd/ST_RATIO)
plot(t,steering_act/ST_RATIO)

xlabel('Time (s)')
ylabel('Steering Angle (rad)')
title('Car'' Commanded Steering Vs. Car''s Actual Steering')

legend('Command','Actual')

subplot(2,1,2)
hold on;
plot(t,steering_cmd/ST_RATIO)
plot(t,delta_f)

xlabel('Time (s)')
ylabel('Steering Angle (rad)')
title('lk\_pcis\_controller Output Vs. Car''s Commanded Steering')

legend('Steering Command','LK PCIS Controller')

disp(' ')
disp('Finished plotting the results of experiment 1 (duplication of commanded steering).')
disp('=================================================================================')

%% Create an Input Trajectory that considers the last input

disp('Experiment 2')
disp('Now adding the second controller.')

LK2 = lk_pcis_controller2;
LK2.H_u = 100;
LK2.setup(struct());

LK3 = lk_pcis_controller;
LK3.H_u = 100;
LK3.setup(struct());

%LK2.redefineBufferLen(1);

for k = t0 : t0+test_duration-1

	k

	if k > t0
		%release(LK2);
		% clear LK2;
		% LK = lk_pcis_controller;
		LK3.H_u = 100;
		LK3.f_u = -LK3.H_u*exp2.LK.delta_f(end);
		-LK3.H_u*exp2.LK.delta_f(end)
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
	[ exp2.LK.delta_f(k), 	exp2.LK.lk_info(k) ] = LK3.step( temp_lk_acc_state );
	[ exp2.LK2.delta_f(k),	exp2.LK2.lk_info(k) ] = LK2.step( temp_lk_acc_state );

	if (k > 10) & (exp2.LK.delta_f(k)~=exp2.LK2.delta_f(k))
		disp([ 'u_LK(t-1) = ' num2str(exp2.LK.delta_f(k-1)) ', u_LK2(t-1) = ' num2str(exp2.LK2.delta_f(k-1)) ])
		disp(['-LK.H_u*exp2.LK.delta_f(k-1) = ' num2str(-LK.H_u*exp2.LK.delta_f(k-1)) ])
		error(['WTF. k = ' num2str(k) ' and u(LK) = ' num2str(exp2.LK.delta_f(k)) ' , u(LK2) = ' num2str(exp2.LK2.delta_f(k)) ])
	end

end

figure;
hold on;
plot(t,delta_f)
plot(t,exp2.LK.delta_f)
plot(t,exp2.LK2.delta_f)

xlabel('Time (s)')
ylabel('Control Input, \delta_f')
title('Trajectories for control inputs when a history-respecting term is added')
legend('Current','Smoothing with u[t-1]','lk_pcis_controller2')

% figure;
% plot(t,exp2.LK2.delta_f)


%%%%%%%%%%%%%%%%%%%%
%% Saving Results %%
%%%%%%%%%%%%%%%%%%%%

results.name = 'mcity_data_practice2';
results.delta_f = delta_f;
results.lk_info = lk_info;
results.exp2 = exp2;
results.t = t;
results.t0 = t0;

end