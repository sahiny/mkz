%% get_ba_message: get Shaobing byte array message
% WARNING: will ONLY behave correctly provided that inputs
% are in specific ranges:
% 	brak \in [0, 1)        
%   thro \in [0, 1)
%   stee \in (-10, 10)
%   gear \in {0, ..., 9}  (uint)
%   turn \in {0, ..., 9}  (uint)
function [msg] = get_ba_message(brak, thro, stee, gear, turn)

	if isempty(brak) || brak < 0 || brak >= 1
		brak = 0.;
	end

	if isempty(thro) || thro < 0 || brak >= 1
		thro = 0.;
	end

	if isempty(stee) || stee <= -10 || stee >= 10
		stee = 0.;
	end

	if isempty(gear) || gear < 0 || gear >= 10
		gear = ps_gear_position_kind.GEAR_POSITION_INVALID;
	end

	if isempty(turn) || turn < 0 || turn >= 10
		turn = ps_platform_turn_signal_kind.PLATFORM_TURN_SIGNAL_INVALID;
	end

	% Ground truth string
	% pub_string = sprintf('OPENCAV:BRAKE%0.3f,THROTTLE%0.3f,STEER%0.3f,GEAR%d,SIGNAL%d,END', ...
		     		      % brak, thro, stee, gear, turn)

	msg = PsByteArrayMessage.zeros();
	msg.Bytes(1:13) = uint8('OPENCAV:BRAKE');

	% brake 0.xxx \in [0,1)
	msg.Bytes(14:15) = uint8('0.');
	msg.Bytes(16:19) = get_ascii_rep(brak, 4);

	msg.Bytes(20:28) = uint8(',THROTTLE');	

	%  throttle 0.xxx \in [0,1)
	msg.Bytes(29:30) = uint8('0.');				
	msg.Bytes(31:34) = get_ascii_rep(thro, 4);		

	%  steer x.yyy \in (-10, 10\)
	msg.Bytes(35:40) = uint8(',STEER');
	if stee > 0
		msg.Bytes(41) = uint8('+');
	else
		msg.Bytes(41) = uint8('-');
	end
	msg.Bytes(42) = uint8('0') + floor(abs(stee));
	msg.Bytes(43) = uint8('.');
	msg.Bytes(44:47) = get_ascii_rep(abs(stee) -  floor(abs(stee)), 4);
	msg.Bytes(48:52) = uint8(',GEAR');
	msg.Bytes(53) = uint8('0') + uint8(gear);

	msg.Bytes(54:60) = uint8(',SIGNAL');
	msg.Bytes(61) = uint8('0') + uint8(turn);

	msg.Bytes(62:65) = uint8(',END');

	msg.BytesSize = uint32(65);
end