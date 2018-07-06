%clear all;close all;clc;
% plots responses of acc and lk pid controllers of chosen experiment
experiment_to_test = 5;
mat_names = {'jun29_test1_possibly'...  %1
        ,'jun29_test2_possibly'...  %2
        ,'jun29_test3_possibly'...  %3
            ,'run-737240.8817'...    %4
            ,'run-737240.8848'...   %5
            , 'run-737240.8871'...  %6
            ,'run-737240.8879'...   %7
            ,'run-737240.8885'...   %8 
            , 'run-737240.8907'...  %9
            , 'run-737240.8909'};   %10

filename = mat_names{experiment_to_test};
load([filename,'.mat']);
new_filename = strrep(filename,'_', '\_');

% setup the controllers
  MU_DES = 25;              % desired forward speed of the car (km/h)
  INPUT_STEP = 0.24;        % desired step-input (+ throttle, - brake)   

  GAINS_ACC = [0.025 0 0]*100;  % K_p, K_i, K_d for ACC
  GAINS_LK = [0.05 0.03 0]*0.8;   % K_p, K_i, K_d for LK
  
  STAB_THRES = 0.5;

  MEM_LENGTH_ACC = 100;
  MEM_LENGTH_LK = 20;

  RTK_SENSOR_ID = 1;    % sensor id of RTK GPS
  STOP_DISTANCE = 20;   % start braking [m]
  ST_RATIO = 12;        % steering ratio of car
  WHEEL_RADIUS = 0.24;  % wheel radius [m]
  STEERING_MAX = 0.78;  % max steering     
  THROTTLE_MAX = 0.28;  % max throttle
  BRAKE_MAX = 0.3;      % maximal braking when stopping
  BRAKE_TIME = 10;      % brake ramp time [s]
  
  
  ACC = acc_pid_controller_test1;
  ACC.mu_des = MU_DES/3.6;
  ACC.K_p = GAINS_ACC(1);
  ACC.K_i = GAINS_ACC(2);
  ACC.K_d = GAINS_ACC(3);
  ACC.max_throttle = THROTTLE_MAX;
  ACC.mem_length = MEM_LENGTH_ACC;
  ACC.throttle_step = INPUT_STEP;
  ACC.stab_thres = STAB_THRES;
  ACC.setup(struct(), 0.0);

  LK = lk_pid_controller_test1;
  LK.K_p = GAINS_LK(1);
  LK.K_i = GAINS_LK(2);
  LK.K_d = GAINS_LK(3);
  LK.mem_length = MEM_LENGTH_LK;
  LK.max_steering = STEERING_MAX;
  LK.setup(struct(), 0.0);
  
  	t = gpsdata.Acceleration.Time;
	accel = gpsdata.Acceleration.Data;
	accel = reshape(accel,size(accel,1),size(accel,3));
	%throttle = data.rawdata.Throttle.Data;

	%Using Petter's Decoding of BA Messages to Get Throttle
	NUM = 70;   % number of bytes to read
	string_vec = char(squeeze(ba_message.Bytes.Data(1:NUM,1,1:end))');
	string_cell = mat2cell(string_vec, ones(1, size(string_vec,1)), NUM);

	pattern = 'OPENCAV:BRAKE%f,THROTTLE%f,STEER%f,GEAR%d,SIGNAL%d,END';

	ba_data_cell = cellfun(@(s) sscanf(s, pattern), string_cell, 'UniformOutput', false);

	ba_data_vec = zeros(5, length(ba_data_cell));
	ba_data_vec(:, ~cellfun(@isempty,ba_data_cell)) = cell2mat(ba_data_cell');

	brake_ba = ba_data_vec(1,:);
	throttle_ba = ba_data_vec(2,:);
	steering_ba = ba_data_vec(3,:);
    
    idx_brake = find(brake_ba,1);
    idx_throttle = find(throttle_ba,1);
    

state = struct();

% simulate
T = length(squeeze(lk_acc_state.y.Time));
dt = lk_acc_state.y.Time(2)-lk_acc_state.y.Time(1);
Steering = zeros(T,1);
Throttle = zeros(T,1);
for i=idx_throttle:T 
    state.y = lk_acc_state.y.Data(i);
    state.dy = lk_acc_state.dy.Data(i);
    state.mu = lk_acc_state.mu.Data(i);
    state.nu = lk_acc_state.nu.Data(i);
    state.dPsi = lk_acc_state.dPsi.Data(i);
    state.r = lk_acc_state.r.Data(i);
    state.h = lk_acc_state.h.Data(i);
    state.r_d = lk_acc_state.r_d.Data(i);
    
    Steering(i) = LK.step(state, dt);
    Throttle(i) = ACC.step(state, dt);
end

% plot LK states
% figure;clf;hold on;
% 
% subplot(411)
y = lk_acc_state.y.Data;
idx_y = find(abs(y)<100);
y_clean = y(idx_y);
% plot(idx_y*0.01, y_clean);
% ylabel('y')
% 
% subplot(412)
% nu = lk_acc_state.nu.Data;
% idx_nu = find(abs(nu)<100);
% nu_clean = nu(idx_nu);
% plot(idx_nu*0.01, nu_clean);
% ylabel('nu')
% 
% subplot(413)
% dPsi = lk_acc_state.dPsi.Data;
% idx_dPsi = find(abs(dPsi)<100);
% dPsi_clean = dPsi(idx_dPsi);
% plot(idx_dPsi*0.01, dPsi_clean);
% ylabel('\delta \Psi')
% 
% subplot(414)
r = lk_acc_state.r.Data;
idx_r = find(abs(r)<100);
r_clean = r(idx_r);
% plot(idx_r*0.01, r_clean);
% ylabel('r')

% figure;clf;
% hold on
% % plot(idx_r*0.01, 10*r_clean,'.');
% % % plot(idx_y*0.01, y_clean,'.');
% % idx_y = idx_y+1;
% % idx_y = idx_y(1:end-1);
% plot(lk_acc_state.y.Time(idx_brake:end), lk_acc_state.y.Data(idx_brake:end),'.');
% plot(lk_acc_state.y.Time(idx_throttle:end), Steering(idx_throttle:end),'.');
% plot(lk_acc_state.y.Time(idx_brake:end), steering_ba(idx_brake:end),'.');
% plot(steering_report.SteeringWheelAngle.Time, squeeze(steering_report.SteeringWheelAngle.Data),'.');
% legend('y','steering_recreated', 'steering_ba', 'steering_report');
% axis([idx_brake*0.01 T*0.01 -10 10])

steering_recreated = Steering;
throttle_recreated = Throttle;
idx_start_test = idx_brake;
idx_start_controllers = idx_throttle;
idx_clean_lk_acc_state = idx_y;
idx_clean_steering = idx_y(1:end-1)+1;

if abs(1-experiment_to_test) < 2
    idx_end_test = 6059;
elseif abs(4-experiment_to_test) < 0.1
    idx_end_test = 9854;
elseif abs(5-experiment_to_test) < 0.1
    idx_end_test = 8876;
end

save(['run', num2str(experiment_to_test)], 'ba_data_vec', 'gpsdata','lk_acc_state','rawdata',...
'steering_report', 'steering_recreated', 'throttle_recreated',...
    'idx_clean_lk_acc_state','idx_clean_steering','idx_start_test','idx_start_controllers','idx_end_test');


