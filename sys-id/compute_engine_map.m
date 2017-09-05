% Attempt to inverte the engine map to obtain (torque, speed) -> throttle

emap = load('engine_map.txt');  % map (throttle, speed) -> torque

n = rawdata.RGear_Tr.Data;		% transmission gear ratio
u = rawdata.Throttle_Eng.Data;	% engine throttle
F_eng = rawdata.M_EngOut.Data;	% engine output torque
F_whl = rawdata.Fx.Data;		% wheel output torque
w = rawdata.AV_Eng.Data;		% engine speed

u_vec = emap(1,2:end);
w_vec = emap(2:end,1);

F_iterp = @(u,w) interp2(u_vec, w_vec, emap(2:end,2:end), u, w);

F_eng_vec = 0:25:500;

throttle_mat = zeros(length(w_vec), length(F_eng_vec));

for i = 1:length(F_eng_vec)
	for j = 1:length(w_vec)
		u0 = fminsearch(@(u) abs(F_iterp(u, w_vec(j)) - F_eng_vec(i)), 0);
		throttle_mat(j, i) = u0;
	end
end

[XX, YY] = meshgrid(F_eng_vec, w_vec);
surf(XX, YY, throttle_mat)

save('engine_map.mat', 'F_eng_vec', 'w_vec', 'throttle_mat')