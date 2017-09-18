load data/run_2

tmin = 15;
tmax = 25;
% tmin = 0;
% tmax = Inf;

tidx = find(and(rawdata.longitude.Time < tmax, ...
				tmin < rawdata.longitude.Time));

% Convert BA message
is_ba = exist('ba_message');
is_df = exist('delta_f');

if is_ba
	brake = str2num(char(squeeze(ba_message.Bytes.Data(14:19,1,1:end))'));
	throttle = str2num(char(squeeze(ba_message.Bytes.Data(29:34,1,1:end))'));
	steering = str2num(char(squeeze(ba_message.Bytes.Data(42:47,1,1:end))'));
end

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


figure(4); clf
if is_df
	idx = find(and(delta_f.Time < tmax, delta_f.Time > tmin))
	subplot(211)
	plot(delta_f.Time(idx), squeeze(delta_f.Data(1,1,idx)))
	subplot(212); hold on
	plot(lk_cinfo.barrier_val.Time(idx), squeeze(lk_cinfo.barrier_val.Data(idx)))
	plot(xlim, [0 0], '-g')
end