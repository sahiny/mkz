%% polysync_controller: controls via polysync bus
function polysync_controller()
	RTK_SENSOR_ID = uint32(1);	% sensor id of RTK GPS
	DT = 1e-2;					% time step [s]
	STOP_DISTANCE = 20;   		% start braking [m]
	STEERING_RATIO = 16.5;		% steering ratio of car

	BRAKE_MAX = 0.3;			% maximal braking when stopping
	BRAKE_TIME = 5;				% brake ramp time [s]

	pub = polysync.Publisher('MessageType', 'ByteArrayMessage');
	pub_cm = polysync.Publisher('MessageType', 'CommandMessage');

	sub_mo = polysync.Subscriber('MessageType', 'PlatformMotionMessage');

	% Set up systems
	rd = road;
	rd.pathfile = '../mcity/mcity_east_lower.ascii';
	rd.circular = 0;
	rd.setup(struct('latitude', 0, 'longitude', 0));

	ACC = acc_pid_controller;
	ACC.max_throttle = 0.28;
	ACC.setup(struct(), DT);

	LK = lk_pcis_controller;
	LK.setup(struct());

	% phase 0: shifting to D
	% phase 1: LK following route
	% phase 2: car stopping
	% phase 3: shifting to P

	% Phase variables
	br_command = 0;    % braking phase

	phase = uint8(0);
	% Shift to D
	shift(pub, ps_gear_position_kind.GEAR_POSITION_DRIVE, DT);

	phase = phase + 1;

	% Control loop
	while phase < uint8(3)
		% Read data
		[idx, sub_msg] = sub_mo.step();
		if idx > 0 && sub_msg.SensorDescriptor.Id == RTK_SENSOR_ID
			rawdata = get_data(sub_msg);

			% Tranform data to model states
			[lk_acc_state, road_left] = rd.step(rawdata);

			if road_left < 20
				phase = phase + 1;
			end

			% Compute model inputs
			[delta_f, lk_info] = LK(lk_acc_state);

			if phase == uint8(1)
				% ACC controls speed
				[throttle_com] = ACC(lk_acc_state, DT)
				pub_msg = get_ba_message(0., throttle_com, ...
					STEERING_RATIO*delta_f, ...
					ps_gear_position_kind.GEAR_POSITION_INVALID, ...
					ps_platform_turn_signal_kind.PLATFORM_TURN_SIGNAL_INVALID);
			elseif phase == uint8(2)
				% Braking phase
				br_command = max(br_command + BRAKE_MAX * DT / BRAKE_TIME, ...
								 BRAKE_MAX);
				pub_msg = get_ba_message(br_command, 0, ...
					STEERING_RATIO*delta_f, ...
					ps_gear_position_kind.GEAR_POSITION_INVALID, ...
					ps_platform_turn_signal_kind.PLATFORM_TURN_SIGNAL_INVALID);
				if abs(rawdata.Vx) <  1e-2
					phase = phase + 1;
				end
			else
				pub_msg = get_ba_message(0,0,0, ...
							ps_gear_position_kind.GEAR_POSITION_INVALID, ...
							ps_platform_turn_signal_kind.PLATFORM_TURN_SIGNAL_INVALID);
			end
					
			pub_msg.Header.Timestamp = polysync.GetTimestamp;

			cinfo_message = PsCommandMessage.zeros();
			cinfo_message.Id = embedded.fi(454545, 'Signed', 0, 'WordLength', 64, ...
									'FractionLength', 0);
			cinfo_message.Data(1).D = ps_parameter_value_kind.PARAMETER_VALUE_DOUBLE;
			% cinfo_message.Data(1).U.DValue = acc_info.barrier_val;
			cinfo_message.Data(2).D = ps_parameter_value_kind.PARAMETER_VALUE_DOUBLE;
			cinfo_message.Data(2).U.DValue = lk_info.barrier_val;
			cinfo_message.DataSize = uint32(2);
			cinfo_message.Header.Timestamp = polysync.GetTimestamp;

			pub.step(pub_msg);
			pub_cm.step(cinfo_message);
			
			polysync.Sleep(DT);
		end
	end

	shift(pub, ps_gear_position_kind.GEAR_POSITION_PARK, DT);
end
