veh_path = load('C:/MKZ/mcity/mcity_outer.ascii');

L_lk = load('lk_pcis_controller.mat');
L_acc = load('acc_pcis_controller.mat');

val = exist('data_lk_validate');

figure(1)
clf; hold on;
subplot(411)
hold on
plot(lk_acc_state.y.Time, lk_acc_state.y.Data)
if val
    plot(data_lk_validate.Time, data_lk_validate.Data(:,1), '--g')
end
plot(xlim, [L_lk.con.y_max L_lk.con.y_max], '--g')
plot(xlim, [-L_lk.con.y_max -L_lk.con.y_max], '--g')
ylim([-1.1*L_lk.con.y_max, 1.1*L_lk.con.y_max]);
ylabel('y')

subplot(412)
hold on
plot(lk_acc_state.nu.Time, lk_acc_state.nu.Data)
if val
    plot(data_lk_validate.Time, data_lk_validate.Data(:,2), '--g')
end
ylim([-1.1*L_lk.con.nu_max 1.1*L_lk.con.nu_max]);
ylabel('nu')

subplot(413)
hold on
plot(lk_acc_state.dPsi.Time, lk_acc_state.dPsi.Data)
if val
    plot(data_lk_validate.Time, data_lk_validate.Data(:,3), '--g')
end
% ylim([-1.1*L_lk.con.psi_max 1.1*L_lk.con.psi_max]);
ylabel('\Delta \Psi')

subplot(414)
hold on
plot(lk_acc_state.r.Time, lk_acc_state.r.Data)
if val
    plot(data_lk_validate.Time, data_lk_validate.Data(:, 4), '--g')
end
ylim([-1.1*L_lk.con.r_max 1.1*L_lk.con.r_max])
ylabel('r')

figure(2); clf
subplot(311)
hold on 
plot(lk_acc_state.mu.Time, lk_acc_state.mu.Data(:))
plot(xlim, [L_acc.con.u_max L_acc.con.u_max], '--g')
plot(xlim, [L_acc.con.u_min L_acc.con.u_min], '--g')
ylabel('mu')
ylim([0, 3*L_acc.con.u_max])

subplot(312)
hold on
plot(rawdata.Fx.Time, rawdata.Fx.Data)
% plot(F_w.Time, F_w.Data(:), 'k--')
plot(xlim, [L_acc.con.Fw_max L_acc.con.Fw_max], '--g')
plot(xlim, [L_acc.con.Fw_min L_acc.con.Fw_min], '--g')
ylim([1.1*L_acc.con.Fw_min, 1.1*L_acc.con.Fw_max])
ylabel('F_w')

subplot(313)
hold on
plot(lk_acc_state.r_d.Time, lk_acc_state.r_d.Data(:))
plot(xlim, [L_lk.con.rd_max L_lk.con.rd_max], '--g')
plot(xlim, [-L_lk.con.rd_max -L_lk.con.rd_max], '--g')
ylim([-2.5*L_lk.con.rd_max, 2.5*L_lk.con.rd_max])
ylabel('r_d')

figure(3)
clf; hold on;
subplot(311)
hold on
plot(delta_f.Time, delta_f.Data(:), 'k--')
plot(rawdata.steer_L1.Time, rawdata.steer_L1.Data, 'b')
plot(rawdata.steer_R1.Time, rawdata.steer_R1.Data, 'r')
plot(xlim, [L_lk.con.df_max L_lk.con.df_max], '--g')
plot(xlim, [-L_lk.con.df_max -L_lk.con.df_max], '--g')
ylim([-1.1*L_lk.con.df_max, 1.1*L_lk.con.df_max])
ylabel('\delta_f')

subplot(312)
hold on
plot(lk_control_info.barrier_val.Time, lk_control_info.barrier_val.Data)
ylim([-0.1, 1.1*max(lk_control_info.barrier_val.Data)])
plot(xlim, [0 0], '--')
ylabel('poly_dist')

% subplot(313)
% hold on
% plot(acc_control_info.barrier_val.Time, acc_control_info.barrier_val.Data)
% ylim([-0.1, 1.1*max(acc_control_info.barrier_val.Data)])
% plot(xlim, [0 0], '--')
% ylabel('poly_dist')


