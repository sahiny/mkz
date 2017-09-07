%% polysync_controller: controls via polysync bus
function polysync_controller()

	dt = 1e-2;
	stop_distance = 20;   % start braking here

	steering_publisher = polysync.Publisher('MessageType', 'PlatformSteeringCommandMessage');
	throttle_publisher = polysync.Publisher('MessageType', 'PlatformThrottleCommandMessage');
	brake_publisher = polysync.Publisher('MessageType', 'PlatformBrakeCommandMessage');

	motion_subscriber = polysync.Subscriber('MessageType', 'PlatformMotionMessage');

	% Set up systems
	OutputHandler = road;
	OutputHandler.pathfile = '../mcity/mcity_outer.ascii';
	OutputHandler.circular = 0;
	OutputHandler.setup(struct('lat', 0, 'long', 0));

	ACC = acc_pcis_controller;
	ACC.setup(struct(), dt);
		steering_message = PsPlatformSteeringCommandMessage.zeros();
		steering_message.Enabled = 1;

	LK = lk_pcis_controller;
	LK.setup(struct());

	BrakeRamp = brake_ramp;
	BrakeRamp.end_ramp = 0.3;    % end brake command
	BrakeRamp.end_time = 5;		 % time to reach max braking command

	InputHandler = car_inputs;
	InputHandler.inv_engine_map_file = 'inverse_engine_map.mat';
	InputHandler.wheel_rad = 0.392;		% wheel radius
	InputHandler.trans_eff = 0.79;  	% power lost engine -> wheels
	InputHandler.brake_gain = 1;		% final brake value: end_ramp * gain
	InputHandler.steer_ratio = 16.5;

	% Control loop
	while true
		% Read data
		[~, msg] = motion_subscriber.step();
		rawdata = MotionMessage_to_rawdata(msg);

		% Tranform data to model states
		[lk_acc_state, road_left] = OutputHandler.step(rawdata);

		% Compute model inputs
		[F_w, ~] = ACC(lk_acc_state, dt);
		[delta_f, ~] = LK(lk_acc_state);

		% Are we close to end?
		brake = BrakeRamp(road_left < stop_distance, dt);
		if brake > 0
			F_w = -brake;
		end

		% Transform model inputs to car inputs
		[steering_com, throttle_com, brake_com] = InputHandler.step(delta_f, F_w, rawdata);

		% Prepeare messages
		steering_message = PsPlatformSteeringCommandMessage.zeros();
		steering_message.Enabled = uint8(1);
		steering_message.SteeringCommandKind = ps_steering_command_kind.STEERING_COMMAND_ANGLE;
		steering_message.SteeringWheelAngle = single(steering_com);
		steering_message.Timestamp = polysync.GetTimestamp;

		throttle_message = PsPlatformThrottleCommandMessage.zeros();
		throttle_message.Enabled = uint8(1);
		throttle_message.ThrottleCommandType = ps_throttle_command_kind.THROTTLE_COMMAND_PERCENT;
		throttle_message.ThrottleCommand = single(throttle_com);
		throttle_message.Timestamp = polysync.GetTimestamp;

		brake_message = PsPlatformBrakeCommandMessage.zeros();
		brake_message.Enabled = uint8(1);
		brake_message.BrakeCommandType = ps_brake_command_kind.BRAKE_COMMAND_PERCENT;
		brake_message.BrakeCommand = single(brake_com);
		brake_message.Timestamp = polysync.GetTimestamp;

		% Apply controls
		steering_publisher.step(steering_message);
		throttle_publisher.step(throttle_message);
		brake_publisher.step(brake_message);

		% Sleep
		polysync.Sleep(dt);
	end
end


%% gps_to_rawdata:
function [rawdata] = MotionMessage_to_rawdata(msg)
	rawdata = struct();
    rawdata.lat = msg.Latitude;					% radians
    rawdata.long = msg.Longitude;				% radians
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