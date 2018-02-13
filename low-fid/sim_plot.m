L_lk = load('lk_pcis_controller.mat');
L_acc = load('acc_pcis_controller.mat');

lk_acc_state = logsout.getElement('lk_acc_state').Values;
delta_f = logsout.getElement('delta_f').Values;
F_w = logsout.getElement('F_w').Values;

figure(1)

clf; hold on;
subplot(411)
hold on
plot(lk_acc_state.y.Time, lk_acc_state.y.Data)
plot(xlim, [L_lk.con.y_max L_lk.con.y_max], '--g')
plot(xlim, [-L_lk.con.y_max -L_lk.con.y_max], '--g')
ylim([-1.1*L_lk.con.y_max, 1.1*L_lk.con.y_max]);
ylabel('y')

subplot(412)
hold on
plot(lk_acc_state.nu.Time, lk_acc_state.nu.Data)
ylim([-1.1*L_lk.con.nu_max 1.1*L_lk.con.nu_max]);
ylabel('nu')

subplot(413)
hold on
plot(lk_acc_state.dPsi.Time, lk_acc_state.dPsi.Data)
% ylim([-1.1*L_lk.con.psi_max 1.1*L_lk.con.psi_max]);
ylabel('\Delta \Psi')

subplot(414)
hold on
plot(lk_acc_state.r.Time, lk_acc_state.r.Data)
ylim([-1.1*L_lk.con.r_max 1.1*L_lk.con.r_max])
ylabel('r')


figure(2); clf

subplot(511)
hold on 
plot(lk_acc_state.mu.Time, lk_acc_state.mu.Data(:))
plot(xlim, [L_acc.con.u_max L_acc.con.u_max], '--g')
plot(xlim, [L_acc.con.u_min L_acc.con.u_min], '--g')
ylabel('mu')
% ylim([0, L_acc.con.u_max])

subplot(512)
hold on
plot(F_w.Time, F_w.Data(:))
plot(xlim, [L_acc.con.Fw_max L_acc.con.Fw_max], '--g')
plot(xlim, [L_acc.con.Fw_min L_acc.con.Fw_min], '--g')
ylim([1.1*L_acc.con.Fw_min, 1.1*L_acc.con.Fw_max])
ylabel('F_w')

subplot(513)
hold on
plot(delta_f.Time, delta_f.Data(:))
plot(xlim, [L_lk.con.df_max L_lk.con.df_max], '--g')
plot(xlim, [-L_lk.con.df_max -L_lk.con.df_max], '--g')
ylim([-2.5*L_lk.con.df_max, 2.5*L_lk.con.df_max])
ylabel('delta_f')

subplot(514)
hold on
plot(lk_acc_state.r_d.Time, lk_acc_state.r_d.Data(:))
plot(xlim, [L_lk.con.rd_max L_lk.con.rd_max], '--g')
plot(xlim, [-L_lk.con.rd_max -L_lk.con.rd_max], '--g')
ylim([-2.5*L_lk.con.rd_max, 2.5*L_lk.con.rd_max])
ylabel('r_d')

subplot(515)
hold on
plot(lk_acc_state.h.Time, lk_acc_state.h.Data(:))
plot(xlim, [L_lk.con.h_min L_lk.con.h_min], '--g')
ylabel('h')

save('lowfid_sim.mat', 'lk_acc_state', 'delta_f', 'F_w')