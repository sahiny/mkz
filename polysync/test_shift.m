pub = polysync.Publisher('MessageType', 'ByteArrayMessage');

shift(pub, ps_gear_position_kind.GEAR_POSITION_DRIVE, 0.01);

shift(pub, ps_gear_position_kind.GEAR_POSITION_PARK, 0.01);