% mcity_data_practice.m
%   This is my first use of Petter's code for driving the Lincoln MKZ
%   autonomously.

clear all;
close all;
clc;

%% Constant and Function Import

ST_RATIO = 16;        % steering ratio of car

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

% Apply PCIS Controller Values in open loop to the LK System

t0 = 1000;

x = double([ [ lk_acc_state.y.Data(t0) ; lk_acc_state.nu.Data(t0) ; lk_acc_state.dPsi.Data(t0) ; lk_acc_state.r.Data(t0) ]; ] );

B_lk = [0; LK.Caf/LK.M; 0; LK.lf*LK.Caf/LK.Iz];
E_lk = [0; 0; -1; 0];

for k = t0 : test_duration

	% lk_state = [ lk_acc_state.y.Data(k) ; lk_acc_state.nu.Data(k) ; lk_acc_state.dPsi.Data(k) ; lk_acc_state.r.Data(k) ];
	mu = lk_acc_state.mu.Data(k);

	%----------------------
	% Create State Matrices
	%----------------------
	A_lk = [0, 1, mu, 0; 
          0, -(LK.Caf+LK.Car)/LK.M/mu, 0, ((LK.lr*LK.Car-LK.lf*LK.Caf)/LK.M/mu - mu); 
          0, 0, 0, 1;
          0, (LK.lr*LK.Car-LK.lf*LK.Caf)/LK.Iz/mu,  0, -(LK.lf^2 * LK.Caf + LK.lr^2 * LK.Car)/LK.Iz/mu];
      
  	dt = Ts; %obj.data.con.dt;

  	A = eye(4) + A_lk * dt + A_lk^2 * dt^2/2 + A_lk^3 * dt^3/3/2;

  	A_int = eye(4)*dt + A_lk * dt^2/2 + A_lk^2 * dt^3/3/2 + A_lk^3 * dt^4/4/3/2;

	B = A_int * B_lk;
	E = A_int * E_lk;


	%------------
	%Obtain Input
	%------------

	delta_f = steering_report.SteeringWheelAngleCommand.Data(k)/ST_RATIO;

	%Create next state update:

	lk_state_dot =  A_lk*x(:,end) + B_lk * delta_f ; %+ E*(mu/) %Assuming that the road is straight.

	x = [ x x(:,end)+lk_state_dot*Ts ];

end

x_meas = [ 	reshape(lk_acc_state.y.Data,1,test_duration) ;
			reshape(lk_acc_state.nu.Data,1,test_duration) ;
			reshape(lk_acc_state.dPsi.Data,1,test_duration) ;
			reshape(lk_acc_state.r.Data,1,test_duration) ];

figure;
for sp_num = 1 : 4
	subplot(2,2,sp_num)
	hold on;
	%plot(x(sp_num,:))
	plot(x_meas(sp_num,[t0:end]))

end