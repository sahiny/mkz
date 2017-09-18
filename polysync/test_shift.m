%% test_shift: shift from P to D and back again
function test_shift()
	dt = 0.01;
	pub = polysync.Publisher('MessageType', 'ByteArrayMessage');

	shift(pub, ps_gear_position_kind.GEAR_POSITION_DRIVE, dt);

	shift(pub, ps_gear_position_kind.GEAR_POSITION_PARK, dt);
end