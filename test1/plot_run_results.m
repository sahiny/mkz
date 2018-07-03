% plot_test_results
%filename = uigetfile();
clear all;close all;clc;
experiment_to_plot = 4;
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
      
filename = mat_names{experiment_to_plot};
load([filename, '.mat']);
new_filename = strrep(filename,'_', '\_');
%Using Petter's Decoding of BA Messages to Get Throttle
t=gpsdata.Acceleration.Time;
accel = gpsdata.Acceleration.Data;
accel = reshape(accel,size(accel,1),size(accel,3));
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


% plot LK states
figure;clf;hold on;

subplot(411)
y = lk_acc_state.y.Data;
idx_y = find(abs(y)<100);
y_clean = y(idx_y);
plot(idx_y*0.01, y_clean);
ylabel('y')

% subplot(412)
% dy = lk_acc_state.dy.Data;
% idx_dy = find(abs(dy)<100);
% dy_clean = dy(idx_dy);
% plot(idx_dy*0.01, dy_clean);
% ylabel('dy')

subplot(412)
nu = lk_acc_state.nu.Data;
idx_nu = find(abs(nu)<100);
nu_clean = nu(idx_nu);
plot(idx_nu*0.01, nu_clean);
ylabel('nu')

subplot(413)
dPsi = lk_acc_state.dPsi.Data;
idx_dPsi = find(abs(dPsi)<100);
dPsi_clean = dPsi(idx_dPsi);
plot(idx_dPsi*0.01, dPsi_clean);
ylabel('\delta \Psi')

subplot(414)
r = lk_acc_state.r.Data;
idx_r = find(abs(r)<100);
r_clean = r(idx_r);
plot(idx_r*0.01, r_clean);
ylabel('r')

suptitle(['LK states for ', new_filename]);


% plot LK related 
figure;clf;hold on;
% subplot(411)
% plot(rawdata.latitude.Time, rawdata.latitude.Data);
% ylabel('rawdata\_latitude');

subplot(411)
y = lk_acc_state.y.Data;
idx_y = find(abs(y)<100);
y_clean = y(idx_y);
plot(idx_y*0.01, y_clean);%hold on;
% plot(steering_report.SteeringWheelAngle.Time, 10*squeeze(steering_report.SteeringWheelAngle.Data));
xlabel('y')

subplot(312)
plot(steering_report.SteeringWheelAngle.Time, squeeze(steering_report.SteeringWheelAngle.Data));
xlabel('SteeringWheelAngle')

subplot(313)
r = lk_acc_state.r.Data;
idx_r = find(abs(r)<100);
plot(idx_r*0.01, r(idx_r));hold on;
plot(steering_report.SteeringWheelAngle.Time, 0.15*squeeze(steering_report.SteeringWheelAngle.Data), '.');
xlabel('r overlayed with 0.25x Steering')

suptitle(['Steering results: ',new_filename])

% plot ACC
figure;clf;hold on;

subplot(211)
mu = lk_acc_state.mu.Data;
idx_mu = find(abs(mu)<100);
mu_clean = mu(idx_mu);
plot(idx_mu*0.01, mu_clean,'.');
xlabel('Ego Speed Provided by lk\_acc\_state')
% 
% subplot(312)
% mu = sqrt(sum(squeeze(gpsdata.Velocity.Data).^2,1));
% idx_mu = 1:length(mu);
% plot(gpsdata.Velocity.Time(idx_mu),  mu(idx_mu));
% xlabel('Ego Speed Provided by gpsdata')
% 
% subplot(413)
% mu = sqrt(squeeze(rawdata.Vx.Data).^2+ squeeze(rawdata.Vx.Data).^2);
% idx_mu = find(abs(mu)<100);
% plot(rawdata.Throttle.Time,  rawdata.Vx.Data);
% xlabel('Ego Speed Provided by rawdata')

subplot(212)
% plot(t(1:end-422),10*throttle(423:end));
plot(t,10*throttle_ba,'.');hold on;
plot(t,10*brake_ba,'.');
legend('throttle', 'brake');

% subplot(313)
% mu = lk_acc_state.mu.Data;
% idx_mu = find(abs(mu)<100);
% mu_clean = mu(idx_mu);
% plot(idx_mu*0.01, mu_clean);hold on;
% % plot(t(1:end-422),10*throttle(423:end));
% plot(t,10*throttle_ba);
% xlabel('overlayed with 10x Throttle')

% figure(4);clf;hold on;
% head = squeeze(gpsdata.Heading.Data);
% head_idx = find(head>0);
% plot(gpsdata.Heading.Time(head_idx), head(head_idx), '*')
% dPsi = lk_acc_state.dPsi.Data;
% idx_dPsi = find(abs(dPsi)<100);
% dPsi_clean = dPsi(idx_dPsi);
% plot(idx_dPsi*0.01, dPsi_clean);
% figure
% plot(gpsdata.Orientation.Time, squeeze(gpsdata.Orientation.Data), '*')

