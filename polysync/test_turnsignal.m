%% test_turnsignal: function descrip_turnsignaltion
dt = 0.01;

pub = polysync.Publisher('MessageType', 'ByteArrayMessage');

for i = 1:dt:5
	msg = get_ba_message([],[],[],[],...
			ps_platform_turn_signal_kind.PLATFORM_TURN_SIGNAL_LEFT);
	msg.Header.Timestamp = polysync.GetTimestamp;
	pub.step(msg);
	polysync.Sleep(dt);
end

for i = 1:dt:5
	msg = get_ba_message([],[],[],[],...
			ps_platform_turn_signal_kind.PLATFORM_TURN_SIGNAL_RIGHT);
	msg.Header.Timestamp = polysync.GetTimestamp;
	pub.step(msg);
	polysync.Sleep(dt);
end
