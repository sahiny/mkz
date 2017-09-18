%% gps_to_rawdata:
function [rawdata] = get_data(msg_mot)
	rawdata = struct();

    normDeg = @(x) pi-mod(pi-x, 2*pi);

    % vector GPS -> CG
    r_gps_cg = [.99; 0];                     

    % rotational speed
    omega = msg_mot.RotationRate(3);

    % velocity at GPS
    v_gps = [msg_mot.Velocity(1);
             msg_mot.Velocity(2)];     
    
    % velocity at CG
    v_cg = v_gps + (r_gps_cg) * omega;

    rawdata.latitude = rad2deg(msg_mot.Latitude);	% latitude of GPS  (deg)
    rawdata.longitude = rad2deg(msg_mot.Longitude);	% longitude of GPS (deg)
    rawdata.el = msg_mot.Altitude;					% altitude of GPS  (m)
    rawdata.Yaw = normDeg(msg_mot.Heading + pi/2);	% yaw angle        (rad)  -- 0 is east, positive counter-clockwise
    rawdata.YawRate = omega;                		% yaw rate         (rad/s)
    rawdata.Vx = v_cg(1);           				% velocity at CG, car x axis   (m/s)
    rawdata.Vy = v_cg(2);           				% velocity at CG, car y axis   (m/s)
    rawdata.x_gps_cg = r_gps_cg(1); 				% position of CG from GPS, car x axis (m)
    rawdata.y_gps_cg = r_gps_cg(2);					% position of CG from GPS, car y axis (m)
    rawdata.steer_L1 = 0;
    rawdata.steer_R1 = 0;
    rawdata.Alpha_L1 = 0;
    rawdata.Alpha_R1 = 0;
    rawdata.Fy_L1 = 0;
    rawdata.Fy_R1 = 0;
    rawdata.Fz_L1 = 0;
    rawdata.Fz_R1 = 0;
    rawdata.Fx  = 0;
    rawdata.Gear_CL = 0;
    rawdata.RGear_Tr = 0;
    rawdata.AV_Eng = 0;
    rawdata.Throttle = 0;
    rawdata.Throttle_Eng = 0;			
    rawdata.M_EngOut = 0;
    rawdata.AVy_L1 = 0;
    rawdata.AVy_L2 = 0;
    rawdata.AVy_R1 = 0;
    rawdata.AVy_R2 = 0;
    rawdata.My_Dr_L1 = 0;
    rawdata.My_Dr_L2 = 0;
    rawdata.My_Dr_R1 = 0;
    rawdata.My_Dr_R2 = 0;
    rawdata.F_pedal = 0;
    rawdata.Bk_pedal = 0;
end