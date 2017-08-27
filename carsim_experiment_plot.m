veh_path = load('C:/MKZ/mcity/mcity_outer.ascii');

end_time = length(data_lk.y.Time)-1;

load 'lk_pcis_controller.mat';

y_max = con.y_max;
delta_f_max = con.df_max;
nu_max = con.nu_max;
r_d_max = con.rd_max;

val = exist('data_lk_validate');

figure(1)
clf; hold on;
subplot(211)
hold on
plot(data_lk.y.Time, data_lk.y.Data)
if val
    plot(data_lk_validate.Time, data_lk_validate.Data(:,1), '--g')
end
plot(xlim, [y_max y_max], '--')
plot(xlim, [-y_max -y_max], '--')
ylim([-1.1*y_max, 1.1*y_max]);
ylabel('y')

subplot(212)
hold on
plot(data_lk.nu.Time, data_lk.nu.Data)
if val
    plot(data_lk_validate.Time, data_lk_validate.Data(:,2), '--g')
end
ylim([-1.1*nu_max 1.1*nu_max]);
ylabel('nu')

figure(2)
clf; hold on;

subplot(311)
hold on
plot(data_lk.dPsi.Time, data_lk.dPsi.Data)
if val
    plot(data_lk_validate.Time, data_lk_validate.Data(:,3), '--g')
end
ylim([-1.1*con.psi_max 1.1*con.psi_max]);
ylabel('\Delta \Psi')

subplot(312)
hold on
plot(data_lk.r.Time, data_lk.r.Data)
if val
    plot(data_lk_validate.Time, data_lk_validate.Data(:, 4), '--g')
end
ylim([-1.1*con.r_max 1.1*con.r_max])
ylabel('r')

subplot(313)
hold on
plot(r_d.Time, r_d.Data(:))
plot(xlim, [r_d_max r_d_max], '--')
plot(xlim, [-r_d_max -r_d_max], '--')
ylim([-2.5*r_d_max, 2.5*r_d_max])

ylabel('r_d')

figure(3)
clf; hold on;
subplot(211)
hold on
plot(delta_f.Time, delta_f.Data(:), 'k--')
plot(data.steer_L1.Time, data.steer_L1.Data, 'b')
plot(data.steer_R1.Time, data.steer_R1.Data, 'r')
plot(xlim, [delta_f_max delta_f_max], '--')
plot(xlim, [-delta_f_max -delta_f_max], '--')
ylim([-1.1*delta_f_max, 1.1*delta_f_max])
ylabel('\delta_f')

subplot(212)
hold on
plot(control_info.barrier_val.Time, control_info.barrier_val.Data)
ylim([-0.1, 1.1*max(control_info.barrier_val.Data)])
plot(xlim, [0 0], '--')
ylabel('poly_dist')

