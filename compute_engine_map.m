emap = load('engine_map.txt');

n = rawdata.RGear_Tr.Data;		% transmission gear ratio
u = rawdata.Throttle_Eng.Data;	% engine throttle
F_eng = rawdata.M_EngOut.Data;	% engine output torque
F_whl = rawdata.Fx.Data;		% wheel output torque
w = rawdata.AV_Eng.Data;		% engine speed

u_vec = emap(1,2:end);
w_vec = emap(2:end,1);

figure(1); clf; hold on
F_eng_interp = interp2(u_vec, w_vec, emap(2:end, 2:end), u, w);
plot(F_whl, '--')
plot(F_eng_interp.*n/0.17)

F_iterp = @(u,w) interp2(u_vec, w_vec, emap(2:end,2:end), u, w);

F_eng_vec = 0:50:500;

map = zeros(length(w_vec), length(F_eng_vec));

for i = 1:length(F_eng_vec)
	for j = 1:length(w_vec)
		u0 = fminsearch(@(u) abs(F_iterp(u, w_vec(j)) - F_eng_vec(i)), 0);
		map(j, i) = u0;
	end
end

u_interp = interp2(F_eng_vec, w_vec, map, F_eng, w)
figure(2); clf; hold on
plot(u_interp)
plot(u, '--')