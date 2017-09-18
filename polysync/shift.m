%% shift: shift to new gear
function shift(pub, new_gear, dt)
	pub_msg = get_ba_message(0.25);
	for t=0:dt:3
		pub_msg.Header.Timestamp = polysync.GetTimestamp;
		pub.step(pub_msg);
		polysync.Sleep(dt);
	end

	pub_msg = get_ba_message(0.25, [], [], new_gear);
	for t=0:dt:3
		pub_msg.Header.Timestamp = polysync.GetTimestamp;
		pub.step(pub_msg);
		polysync.Sleep(dt);
	end

	pub_msg = get_ba_message(0.25);
	for t=0:dt:2
		pub_msg.Header.Timestamp = polysync.GetTimestamp;
		pub.step(pub_msg);
		polysync.Sleep(dt);
	end
end
