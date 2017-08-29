steering_publisher = polysync.Publisher('MessageType', 'PlatformSteeringCommandMessage');
throttle_publisher = polysync.Publisher('MessageType', 'PlatformThrottleCommandMessage');

gps_subscriber = polysync.Subscriber('MessageType', 'PlatformMotionMessage');


% Set up systems
Road = road;
Road.pathfile = 'mcity/fixed_path.ascii';
Road.setup(0);

ACC = acc_pcis_controller;
ACC.setup(0);

LK = lk_pcis_controller;
LK.setup(0);

% Control loop
for i=1:4

	% Read data
	[idx, msg] = gps_subscriber.step();
	rawdata = MotionMessage_to_rawdata(msg);

	% Tranform data
	road_s = Road.step(rawdata);
	lk_acc_state = get_state(road_s, rawdata);

	% Compute controls
	[F_w, ~] = ACC(lk_acc_state);
	[delta_f, ~] = LK(lk_acc_state);

	% Apply controls
	steering_message = PsPlatformSteeringCommandMessage.zeros();
end


%% gps_to_rawdata:
function [rawdata] = MotionMessage_to_rawdata(msg)
	rawdata = Simulink.Bus.createMATLABStruct('DataBus');
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