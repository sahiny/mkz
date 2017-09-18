%% gps_to_rawdata:
function [rawdata] = get_data(msg_mot)
	rawdata = struct();

    normDeg = @(x) pi-mod(pi-x, 2*pi);

    % vector GPS -> CG    (IS THIS PLUS OR MINUS??)
    r_gps_cg = [-.99; 0];                     

    % rotational speed
    omega = msg_mot.RotationRate(3);

    % velocity at GPS
    v_gps = [msg_mot.Velocity(1);
             msg_mot.Velocity(2)];     
    
    % velocity at CG
    v_cg = v_gps + (r_gps_cg) * omega;

    rawdata.latitude = rad2deg(msg_mot.Latitude);	% latitude of GPS  (deg)
    rawdata.longitude = rad2deg(msg_mot.Longitude);	% longitude of GPS (deg)
    rawdata.el = msg_mot.Altitude;					% altitude of GPS  (deg)
    rawdata.Yaw = normdeg(msg_mot.Heading - pi/2);	% yaw angle        (rad)  -- 0 is east, positive counter-clockwise
    rawdata.YawRate = omega;                		% yaw rate         (rad/s)
    rawdata.Vx = v_cg(1);           				% velocity at CG, car x axis   (m/s)
    rawdata.Vy = v_cg(2);           				% velocity at CG, car y axis   (m/s)
    rawdata.x_gps_cg = r_gps_cg(1); 				% position of CG from GPS, car x axis (m)
    rawdata.y_gps_cg = r_gps_cg(2);					% position of CG from GPS, car y axis (m)
    rawdata.steer_L1 = NaN;
    rawdata.steer_R1 = NaN;
    rawdata.Alpha_L1 = NaN;
    rawdata.Alpha_R1 = NaN;
    rawdata.Fy_L1 = NaN;
    rawdata.Fy_R1 = NaN;
    rawdata.Fz_L1 = NaN;
    rawdata.Fz_R1 = NaN;
    rawdata.Fx  = NaN;
    rawdata.Gear_CL = NaN;
    rawdata.RGear_Tr = NaN;
    rawdata.AV_Eng = NaN;
    rawdata.Throttle = NaN;
    rawdata.Throttle_Eng = NaN;			
    rawdata.M_EngOut = NaN;
    rawdata.AVy_L1 = NaN;
    rawdata.AVy_L2 = NaN;
    rawdata.AVy_R1 = NaN;
    rawdata.AVy_R2 = NaN;
    rawdata.My_Dr_L1 = NaN;
    rawdata.My_Dr_L2 = NaN;
    rawdata.My_Dr_R1 = NaN;
    rawdata.My_Dr_R2 = NaN;
    rawdata.F_pedal = NaN;
    rawdata.Bk_pedal = NaN;
end