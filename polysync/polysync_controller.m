%% polysync_controller: controls via polysync bus
function polysync_controller()
	dt = 1e-2;
	stop_distance = 20;   % start braking [m]

	pub = polysync.Publisher('MessageType', 'ByteArrayMessage');
	pub_cm = polysync.Publisher('MessageType', 'CommandMessage');

	sub_mo = polysync.Subscriber('MessageType', 'PlatformMotionMessage');
	sub_mo.SourceGuid = 1688903407881414;

	% Set up systems
	trans_out = road;
	trans_out.pathfile = '../mcity/mcity_east_lower.ascii';
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
		rawdata = get_data(msg)

		% Tranform data to model states
		[lk_acc_state, road_left] = trans_out.step(rawdata)

		% Compute model inputs
		[F_w, acc_info] = ACC(lk_acc_state, dt);
		[delta_f, lk_info] = LK(lk_acc_state);

		% Transform model inputs to car inputs
		[steering_com, throttle_com, brake_com] = trans_in.step(delta_f, F_w, rawdata);

		% Are we close to end?
		brake_Fw = BrakeRamp(road_left < stop_distance, dt);
		if brake_Fw > 0
			brake_com = brake_Fw;
			throttle_com = 0.;
		else
			brake_com = 0.;
			throttle_com = 0.1;
		end

		msg = get_ba_message(brake_com, throttle_com, steering_com, ...
				ps_gear_position_kind.GEAR_POSITION_INVALID, ...
				ps_platform_turn_signal_kind.PLATFORM_TURN_SIGNAL_INVALID);
		msg.Header.Timestamp = polysync.GetTimestamp;
		pub.step(msg);

		cinfo_message = PsCommandMessage.zeros();
		cinfo_message.Id = embedded.fi(454545, 'Signed', 0, 'WordLength', 64, ...
								'FractionLength', 0);
		cinfo_message.Data(1).D = ps_parameter_value_kind.PARAMETER_VALUE_DOUBLE;
		cinfo_message.Data(1).U.DValue = acc_info.barrier_val;
		cinfo_message.Data(2).D = ps_parameter_value_kind.PARAMETER_VALUE_DOUBLE;
		cinfo_message.Data(2).U.DValue = lk_info.barrier_val;
		cinfo_message.Data(3).D = ps_parameter_value_kind.PARAMETER_VALUE_DOUBLE;
		cinfo_message.Data(3).U.DValue = road_left;
		cinfo_message.DataSize = uint32(3);

		% Apply controls
		pub_cm.step(cinfo_message);

	end
end

