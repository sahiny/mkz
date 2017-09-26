
tidx = abs(lk_acc_state.y.Data) < 5; 

figure(1)
clf; hold on;
subplot(411)
hold on
plot(lk_acc_state.y.Time(tidx), lk_acc_state.y.Data(tidx))
ylabel('y')

subplot(412)
hold on
plot(lk_acc_state.nu.Time, lk_acc_state.nu.Data)
ylabel('nu')

subplot(413)
hold on
plot(lk_acc_state.dPsi.Time(tidx), lk_acc_state.dPsi.Data(tidx))
ylabel('\Delta \Psi')

subplot(414)
hold on
plot(lk_acc_state.r.Time, lk_acc_state.r.Data)
ylabel('r')

figure(2); clf
subplot(311)
hold on 
plot(lk_acc_state.mu.Time, lk_acc_state.mu.Data(:))
ylabel('mu')


subplot(313)
hold on
plot(lk_acc_state.r_d.Time, lk_acc_state.r_d.Data(:))
ylabel('r_d')

figure(3)
clf; hold on;
hold on
plot(delta_f.Time, squeeze(delta_f.Data))

