%% get_ascii_rep(num, len): for num=0.xxxxxxx return ascii_code of length 'len'
% 	such that char(ascii_code) = 'xxxx'
function [ascii_code] = get_ascii_rep(num, len)
	assert(num < 1)
	ascii_code = uint8(zeros(1,len));
	for i = 1:len
		digit = max(0, floor(10^(i) * num));
		ascii_code(i) = uint8('0') + digit;
		num = num - 10^(-i) * digit;
	end
end
