%% shift: shift to new gear
function shift(pub, new_gear, dt)
	pub_msg = get_ba_message(0.2, 0, 0, ...
				ps_gear_position_kind.GEAR_POSITION_INVALID, ...
				ps_platform_turn_signal_kind.PLATFORM_TURN_SIGNAL_INVALID);
	for t=0:dt:3
		pub_msg.Header.Timestamp = polysync.GetTimestamp;
		pub.step(pub_msg);
		polysync.Sleep(dt);
	end

	pub_msg = get_ba_message(0.2, 0, 0, ...
				new_gear, ...
				ps_platform_turn_signal_kind.PLATFORM_TURN_SIGNAL_INVALID);
	for t=0:dt:3
		pub_msg.Header.Timestamp = polysync.GetTimestamp;
		pub.step(pub_msg);
		polysync.Sleep(dt);
	end

	pub_msg = get_ba_message(0, 0, 0, ...
				ps_gear_position_kind.GEAR_POSITION_INVALID, ...
				ps_platform_turn_signal_kind.PLATFORM_TURN_SIGNAL_INVALID);
	for t=0:dt:3
		pub_msg.Header.Timestamp = polysync.GetTimestamp;
		pub.step(pub_msg);
		polysync.Sleep(dt);
	end
end
