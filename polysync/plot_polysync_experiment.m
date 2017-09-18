load run_3

tmin = 20;
tmax = 30;

tidx = find(and(rawdata.longitude.Time < tmax, ...
				tmin < rawdata.longitude.Time));

% Convert BA message
brake = str2num(char(squeeze(ba_message.Bytes.Data(14:19,1,1:end))'));
throttle = str2num(char(squeeze(ba_message.Bytes.Data(29:34,1,1:end))'));
steering = str2num(char(squeeze(ba_message.Bytes.Data(42:47,1,1:end))'));


figure(1); clf
plot(rawdata.longitude.Data(tidx), rawdata.latitude.Data(tidx))

figure(2)
clf; hold on;
subplot(311)
hold on
plot(lk_acc_state.y.Time(tidx), lk_acc_state.y.Data(tidx))
ylim([-1 1]);
ylabel('y')

subplot(312)
hold on
plot(lk_acc_state.nu.Time(tidx), lk_acc_state.nu.Data(tidx))
ylim([-1 1]);
ylabel('nu')

subplot(313)
hold on
plot(lk_acc_state.dPsi.Time(tidx), lk_acc_state.dPsi.Data(tidx))
ylim([-0.5 0.5]);
ylabel('\Delta \Psi')

figure(3); clf
subplot(211)
hold on
plot(lk_acc_state.r.Time(tidx), lk_acc_state.r.Data(tidx))
ylim([-1 1])
ylabel('r')

subplot(212)
hold on 
plot(lk_acc_state.mu.Time(tidx), lk_acc_state.mu.Data(tidx))
ylabel('mu')
ylim([0, 10])

