% mcity_data_practice.m
%   This is my first use of Petter's code for driving the Lincoln MKZ
%   autonomously.

clear all;
close all;
clc;

%% Constant and Function Import

%Example Dataset
load('data/run-successful.mat');
%load('data/run-snake1.mat');

%Import functions
if ~any(strcmp(path,'../systems/'))
	addpath('../systems/')
end

%% Plot run(s) of the system using GPS Data

%Because the position calculation appears to be messy let's try to use the 
%velocity measurements to integrate and find x.
x0 = zeros(3,1);
Ts = gpsdata.Velocity.TimeInfo.Increment;

x = [x0];
for k = 1:length(gpsdata.Velocity.Time)
    x = [x x(:,end)+gpsdata.Velocity.Data(:,1,k)*Ts];
end

f1 = figure;
for dim = 1:3
    dim_data = x(dim,:);
    subplot(1,3,dim)
    plot(reshape(dim_data,prod( size( dim_data ) ),1 ))
end

f2 = figure;
scatter3(x(1,:),x(2,:),x(3,:))

%% Using Petter's Classes

LK = lk_pcis_controller;
LK.H_u = 4;   % weight in QP for steering (larger -> less aggressive centering)
LK.setup(struct());

ACC = acc_pid_controller;
ACC.mu_des = 26/3.6;
ACC.max_throttle = 0.28;
ACC.setup(struct(), 0.0);

% Apply pcis controller to Petter's 

test_duration = length(lk_acc_state.nu.Data);

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
	%delta_f(k) = LK.step( temp_lk_acc_state );
	throttle(k) = ACC.step( temp_lk_acc_state , Ts );


end

