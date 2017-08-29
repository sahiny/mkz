%% get_state(): get acc/lk state from GPS data and road state
 function [lk_acc_state] = get_state(road_state, data)
    % unit vector direction of road
    road_unit = [road_state.d_road_x; road_state.d_road_y]/...
                norm([road_state.d_road_x; road_state.d_road_y]);

    % relative position
    rel_pos = [road_state.car_x - road_state.road_x;
               road_state.car_y - road_state.road_y];

    % velocity in global frame
    global_vel = [data.Vx*cos(data.Yaw)-data.Vy*sin(data.Yaw);
                  data.Vx*sin(data.Yaw)+data.Vy*cos(data.Yaw)];

    % yaw angles
    normDeg = @(x) pi-mod(pi-x, 2*pi);

    road_yaw = cart2pol(road_unit(1), road_unit(2));
    global_yaw = normDeg(normDeg(data.Yaw) + cart2pol(data.Vx, data.Vy));

    dPsi = normDeg(global_yaw - road_yaw);

    % assign
    lk_acc_state = {};
    lk_acc_state.y = det([road_unit rel_pos]);
    lk_acc_state.dy = det([road_unit global_vel]);
    lk_acc_state.mu = data.Vx;
    lk_acc_state.nu = data.Vy;
    lk_acc_state.dPsi = dPsi;
    lk_acc_state.r = data.YawRate;
    lk_acc_state.h = 8;
    lk_acc_state.r_d = road_state.kappa * norm(global_vel);
end