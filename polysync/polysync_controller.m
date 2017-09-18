dt = 1e-2;

%% polysync_controller: controls via polysync bus
function polysync_controller()
	stop_distance = 20;   % start braking [m]

	pub = polysync.Publisher('MessageType', 'ByteArrayMessage');
	pub_cm = polysync.Publisher('MessageType', 'CommandMessage');

	sub_mo = polysync.Subscriber('MessageType', 'PlatformMotionMessage');
	% should only get from sensor ID 1

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

	% Shift to D
	msg = get_ba_message(0., throttle_com, 16.5*delta_f, ...
			ps_gear_position_kind.GEAR_POSITION_DRIVE, ...
			ps_platform_turn_signal_kind.PLATFORM_TURN_SIGNAL_INVALID);

	% phase 0: shifting to D
	% phase 1: LK following route
	% phase 2: car stopping
	% phase 3: shifting to P

	% Phase variables
	br_command = 0;    % braking phase

	phase = 0;
	shift(pub, ps_gear_position_kind.GEAR_POSITION_DRIVE);

	phase = 1;

	% Control loop
	while phase < 3
		% Read data
		[~, msg] = sub_mo.step();
		rawdata = get_data(msg)

		% Tranform data to model states
		[lk_acc_state, road_left] = trans_out.step(rawdata)

		if road_left < 20
			phase = 2;
		end

		% Compute model inputs
		[delta_f, lk_info] = LK(lk_acc_state);

		if phase == 1
			msg = get_ba_message(0., throttle_com, 16.5*delta_f, ...
				ps_gear_position_kind.GEAR_POSITION_DRIVE, ...
				ps_platform_turn_signal_kind.PLATFORM_TURN_SIGNAL_INVALID);
		elseif phase == 2
			br_command = max(br_command + 0.3 * dt / 5, 0.3);
			pub_msg = get_ba_message(br_command, 0, 16.5*delta_f, ...
				ps_gear_position_kind.GEAR_POSITION_DRIVE, ...
				ps_platform_turn_signal_kind.PLATFORM_TURN_SIGNAL_INVALID);
			if abs(rawdata.Vx) <  1e-2
				phase = 3;
			end
		end
				
		msg.Header.Timestamp = polysync.GetTimestamp;

		cinfo_message = PsCommandMessage.zeros();
		cinfo_message.Id = embedded.fi(454545, 'Signed', 0, 'WordLength', 64, ...
								'FractionLength', 0);
		cinfo_message.Data(1).D = ps_parameter_value_kind.PARAMETER_VALUE_DOUBLE;
		cinfo_message.Data(1).U.DValue = acc_info.barrier_val;
		cinfo_message.Data(2).D = ps_parameter_value_kind.PARAMETER_VALUE_DOUBLE;
		cinfo_message.Data(2).U.DValue = lk_info.barrier_val;
		cinfo_message.Data(3).D = ps_parameter_value_kind.PARAMETER_VALUE_DOUBLE;
		cinfo_message.Data(3).U.DValue = road_left;
		cinfo_message.Data(4).D = ps_parameter_value_kind.PARAMETER_VALUE_DOUBLE;
		cinfo_message.Data(4).U.DValue = lk_acc_state.y;
		cinfo_message.Data(5).D = ps_parameter_value_kind.PARAMETER_VALUE_DOUBLE;
		cinfo_message.Data(5).U.DValue = lk_acc_state.nu;
		cinfo_message.Data(6).D = ps_parameter_value_kind.PARAMETER_VALUE_DOUBLE;
		cinfo_message.Data(6).U.DValue = lk_acc_state.mu;
		cinfo_message.Data(7).D = ps_parameter_value_kind.PARAMETER_VALUE_DOUBLE;
		cinfo_message.Data(7).U.DValue = lk_acc_state.dPsi;
		cinfo_message.Data(8).D = ps_parameter_value_kind.PARAMETER_VALUE_DOUBLE;
		cinfo_message.Data(8).U.DValue = lk_acc_state.r;
		cinfo_message.Data(9).D = ps_parameter_value_kind.PARAMETER_VALUE_DOUBLE;
		cinfo_message.Data(9).U.DValue = lk_acc_state.r_d;
		cinfo_message.DataSize = uint32(9);
		cinfo_message.Header.Timestamp = polysync.GetTimestamp;

		pub.step(msg);
		pub_cm.step(cinfo_message);

		polysync.Sleep(dt);
	end

	shift(ps_gear_position_kind.GEAR_POSITION_PARK);
end

%% shift: shift to new gear
function shift(pub, new_gear)
	pub_msg = get_ba_message(0.2, 0, 0, ...
				ps_gear_position_kind.GEAR_POSITION_INVALID, ...
				ps_platform_turn_signal_kind.PLATFORM_TURN_SIGNAL_INVALID);
	for t=0:dt:2
		pub.step(pub_msg);
		polysync.Sleep(dt);
	end

	pub_msg = get_ba_message(0.2, 0, 0, ...
				new_gear, ...
				ps_platform_turn_signal_kind.PLATFORM_TURN_SIGNAL_INVALID);
	for t=0:dt:2
		pub.step(pub_msg);
		polysync.Sleep(dt);
	end

	pub_msg = get_ba_message(0, 0, 0, ...
				ps_gear_position_kind.GEAR_POSITION_INVALID, ...
				ps_platform_turn_signal_kind.PLATFORM_TURN_SIGNAL_INVALID);
	for t=0:dt:2
		pub.step(pub_msg);
		polysync.Sleep(dt);
	end
end
