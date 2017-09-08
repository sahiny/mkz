%% polysync_controller: controls via polysync bus
function polysync_controller()
	dt = 1e-2;
	stop_distance = 20;   % start braking [m]

	pub_st = polysync.Publisher('MessageType', 'PlatformSteeringCommandMessage');
	pub_th = polysync.Publisher('MessageType', 'PlatformThrottleCommandMessage');
	pub_br = polysync.Publisher('MessageType', 'PlatformBrakeCommandMessage');
	pub_cm = polysync.Publisher('MessageType', 'CommandMessage');

	sub_mo = polysync.Subscriber('MessageType', 'PlatformMotionMessage');

	% Set up systems
	trans_out = road;
	trans_out.pathfile = '../mcity/mcity_outer.ascii';
	trans_out.circular = 0;
	trans_out.setup(struct('latitude', 0, 'longitude', 0));

	ACC = acc_pcis_controller;
	ACC.setup(struct(), dt);

	LK = lk_pcis_controller;
	LK.setup(struct());

	BrakeRamp = brake_ramp;
	BrakeRamp.end_ramp = 0.3;    % end brake command
	BrakeRamp.end_time = 5;		 % time to reach max braking command
	BrakeRamp.setup(false, 0);

	trans_in = car_inputs;
	trans_in.inv_engine_map_file = 'inverse_engine_map.mat';
	trans_in.wheel_rad = 0.392;		% wheel radius
	trans_in.trans_eff = 0.79;  	% power lost engine -> wheels
	trans_in.brake_gain = 1;		% final brake value: end_ramp * gain
	trans_in.steer_ratio = 16.5;
	trans_in.setup(0,0,struct('AVy_L1', 0, 'AV_Eng', 0));

	% Control loop
	while 1
		% Read data
		[~, msg] = sub_mo.step();
		rawdata = MotionMessage_to_rawdata(msg);

		% Tranform data to model states
		[lk_acc_state, road_left] = trans_out.step(rawdata);

		% Compute model inputs
		[F_w, acc_info] = ACC(lk_acc_state, dt);
		[delta_f, lk_info] = LK(lk_acc_state);

		% Are we close to end?
		brake = BrakeRamp(road_left < stop_distance, dt);
		if brake > 0
			F_w = -brake;
		end

		% Transform model inputs to car inputs
		[steering_com, throttle_com, brake_com] = trans_in.step(delta_f, F_w, rawdata);

		assert (throttle_com * brake_com == 0)

		% Prepeare messages
		steering_message = PsPlatformSteeringCommandMessage.zeros();
		steering_message.Enabled = uint8(1);
		steering_message.SteeringCommandKind = ps_steering_command_kind.STEERING_COMMAND_ANGLE;
		steering_message.SteeringWheelAngle = single(steering_com);
		steering_message.Timestamp = polysync.GetTimestamp;

		throttle_message = PsPlatformThrottleCommandMessage.zeros();
		if (throttle_com > 0)
			throttle_message.Enabled = uint8(1);
		end
		throttle_message.ThrottleCommandType = ps_throttle_command_kind.THROTTLE_COMMAND_PERCENT;
		throttle_message.ThrottleCommand = single(throttle_com);
		throttle_message.Timestamp = polysync.GetTimestamp;

		brake_message = PsPlatformBrakeCommandMessage.zeros();
		if (brake_com > 0)
			brake_message.Enabled = uint8(1);
			brake_message.BooEnabled = uint8(1); % brake lights
		end
		brake_message.BrakeCommandType = ps_brake_command_kind.BRAKE_COMMAND_PERCENT;
		brake_message.BrakeCommand = single(brake_com);
		brake_message.Timestamp = polysync.GetTimestamp;

		cinfo_message = PsCommandMessage.zeros();
		cinfo_message.Id = embedded.fi(454545, 'Signed', 0, 'WordLength', 64, ...
								'FractionLength', 0);
		cinfo_message.Data(1).D = ps_parameter_value_kind.PARAMETER_VALUE_DOUBLE;
		cinfo_message.Data(1).U.DValue = acc_info.barrier_val;
		cinfo_message.Data(2).D = ps_parameter_value_kind.PARAMETER_VALUE_DOUBLE;
		cinfo_message.Data(2).U.DValue = lk_info.barrier_val;
		cinfo_message.DataSize = uint32(2);

		% Apply controls
		pub_st.step(steering_message);
		pub_th.step(throttle_message);
		pub_br.step(brake_message);
		pub_cm.step(cinfo_message);

		% Sleep
		polysync.Sleep(dt);
	end
end

%% gps_to_rawdata:
function [rawdata] = MotionMessage_to_rawdata(msg)
	rawdata = struct();
    rawdata.latitude = msg.Latitude;			% radians
    rawdata.longitude = msg.Longitude;			% radians
    rawdata.el = msg.Altitude;					% m
    rawdata.Yaw = msg.Heading;					% radians
    rawdata.YawRate = msg.RotationRate(3);		% rad/s
    rawdata.Vx = msg.Velocity(1);				% m/s
    rawdata.Vy = msg.Velocity(2);				% m/s
    rawdata.x_CG = -0.99;						% m
    rawdata.y_CG = NaN;							% m
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
    rawdata.AV_Eng = 0;     					% rpm
    rawdata.Throttle = 0;						% [0, 1]
    rawdata.Throttle_Eng = NaN;			
    rawdata.M_EngOut = NaN;
    rawdata.AVy_L1 = 0;							% rpm
    rawdata.AVy_L2 = 0;
    rawdata.AVy_R1 = 0;
    rawdata.AVy_R2 = 0;
    rawdata.My_Dr_L1 = NaN;
    rawdata.My_Dr_L2 = NaN;
    rawdata.My_Dr_R1 = NaN;
    rawdata.My_Dr_R2 = NaN;
    rawdata.F_pedal = 0;
    rawdata.Bk_pedal = 0;
end