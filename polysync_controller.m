%% polysync_controller: controls via polysync bus
function polysync_controller()

	steering_publisher = polysync.Publisher('MessageType', 'PlatformSteeringCommandMessage');
	throttle_publisher = polysync.Publisher('MessageType', 'PlatformThrottleCommandMessage');

	motion_subscriber = polysync.Subscriber('MessageType', 'PlatformMotionMessage');


	% Set up systems
	Road = road;
	Road.pathfile = 'mcity/fixed_path.ascii';
	Road.setup(struct('lat', 0, 'long', 0));

	ACC = acc_pcis_controller;
	ACC.setup(struct());

	LK = lk_pcis_controller;
	LK.setup(struct());

	% Control loop
	while true

		% Read data
		[~, msg] = motion_subscriber.step();
		rawdata = MotionMessage_to_rawdata(msg);

		% Tranform data
		lk_acc_state = Road.step(rawdata);

		% Compute controls
		[F_w, ~] = ACC(lk_acc_state);
		[delta_f, ~] = LK(lk_acc_state);

		% Engine mapping
		throttle = 0.065 + 3e-5*F_w;

		% Prepeare messages
		steering_message = PsPlatformSteeringCommandMessage.zeros();
		steering_message.SteeringCommandKind = ps_steering_command_kind.STEERING_COMMAND_ANGLE;
		steering_message.SteeringWheelAngle = single(delta_f);
		steering_message.Timestamp = polysync.GetTimestamp;

		throttle_message = PsPlatformThrottleCommandMessage.zeros();
		throttle_message.ThrottleCommandType = ps_throttle_command_kind.THROTTLE_COMMAND_PERCENT;
		throttle_message.ThrottleCommand = single(throttle);
		throttle_message.Timestamp = polysync.GetTimestamp;

		% Apply controls
		steering_publisher.step(steering_message);
		throttle_publisher.step(throttle_message);

		% Sleep
		polysync.Sleep(1e-3);
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
    rawdata.y_CG = 0;							% m
    rawdata.steer_L1 = 0;
    rawdata.steer_R1 = 0;
    rawdata.Alpha_L1 = 0;
    rawdata.Alpha_R1 = 0;
    rawdata.Fy_L1 = 0;
    rawdata.Fy_R1 = 0;
    rawdata.Fz_L1 = 0;
    rawdata.Fz_R1 = 0;
    rawdata.Fx  = 0;
end