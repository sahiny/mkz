%% get_ba_message(brak, thro, stee, gear, turn): get
% Shaobing-style ByteArray message.
% WARNING: will ONLY behave correctly provided that inputs
% are in specific ranges:
%   brak \in [0, 1)
%   thro \in [0, 1)
%   stee \in (-10, 10)
%   gear \in {0, ..., 9}  (uint)
%   turn \in {0, ..., 9}  (uint)
function [msg] = get_ba_message(brak, thro, stee, gear, turn)

	if nargin < 5
		turn = [];
	end

	if nargin < 4
		gear = [];
	end

	if nargin < 3
		stee = [];
	end

	if nargin < 2
		thro = [];
	end

	if nargin < 1
		brak = [];
	end

	if isempty(brak) || brak < 0 || brak >= 1
		brak_c = 0.;
	else
		brak_c = brak;
	end

	if isempty(thro) || thro < 0 || thro >= 1
		thro_c = 0.;
	else
		thro_c = thro;
	end

	if isempty(stee) || stee <= -10 || stee >= 10
		stee_c = 0.;
	else
		stee_c = stee;
	end

	if isempty(gear) || gear < 0 || gear >= 10
		gear_c = ps_gear_position_kind.GEAR_POSITION_INVALID;
	else
		gear_c = gear;
	end

	if isempty(turn) || turn < 0 || turn >= 10
		turn_c = ps_platform_turn_signal_kind.PLATFORM_TURN_SIGNAL_INVALID;
	else
		turn_c = turn;
	end

	% Ground truth string
	% pub_string = sprintf('OPENCAV:BRAKE%0.3f,THROTTLE%0.3f,STEER%0.3f,GEAR%d,SIGNAL%d,END', ...
		     		      % brak, thro, stee, gear, turn)

	msg = PsByteArrayMessage.zeros();

	head = [uint8('OPENCAV:BRAKE0.') ...
		    get_ascii_rep(brak_c, 4) ...
		    uint8(',THROTTLE0.') ...
			get_ascii_rep(thro_c, 4) ...
			uint8(',STEER')];

	tail = [uint8('0')+floor(abs(stee_c)) uint8('.') ...
			get_ascii_rep(abs(stee_c) -  floor(abs(stee_c)), 4) ...
			uint8(',GEAR') ...
			uint8('0')+uint8(gear_c) ...
			uint8(',SIGNAL') ...
			uint8('0')+uint8(turn_c) ...
			uint8(',END')];

	if stee > 0
		msg.Bytes(1:40) = head;
		msg.Bytes(41:64) = tail;
	else
		msg.Bytes(1:40) = head;
		msg.Bytes(41) = uint8('-');
		msg.Bytes(42:65) = tail;
	end

	msg.BytesSize = uint32(65);
end