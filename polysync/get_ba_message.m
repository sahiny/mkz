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

	head = [uint8('OPENCAV:BRAKE0.') ...
		    get_ascii_rep(brak, 4) ...
		    uint8(',THROTTLE0.') ...
			get_ascii_rep(thro, 4) ...
			uint8(',STEER')];

	tail = [uint8('0')+floor(abs(stee)) uint8('.') ...
			get_ascii_rep(abs(stee) -  floor(abs(stee)), 4) ...
			uint8(',GEAR') ...
			uint8('0')+uint8(gear) ...
			uint8(',SIGNAL') ...
			uint8('0')+uint8(turn) ...
			uint8(',END')];

	if stee > 0
		msg.Bytes(1:40) = head;
		msg.Bytes(41:64) = tail;
		msg.BytesSize = 64;
	else
		msg.Bytes(1:40) = head;
		msg.Bytes(41) = uint8('-');
		msg.Bytes(42:65) = tail;
	end

	msg.BytesSize = uint32(65);
end