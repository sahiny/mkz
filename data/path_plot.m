refSphere = wgs84Ellipsoid;

r = road;
r.pathfile = 'mcity/mcity_outer.ascii';
r.setup(Simulink.Bus.createMATLABStruct('DataBus'));

s_vec = 0:0.5:1200;

[s_cell, xs_cell, ys_cell] = arrayfun(@(s) r.get_pos(s), s_vec, ...
									  'UniformOutput', false);

[rc, drc, kappa] = cellfun(@(s, xs, ys) r.get_road_state(s, xs, ys), ...
					  s_cell, xs_cell, ys_cell, ...
					  'UniformOutput', false);
rc = cell2mat(rc);
drc = cell2mat(drc)';
kappa = cell2mat(kappa);

tt = 0.0001*[-drc(:,2) drc(:,1)]./sqrt(sum(drc.*drc, 2));

[lat, long, h] = enu2geodetic(rc(1,:), rc(2,:), 0, r.lat0, r.long0, r.h0, wgs84Ellipsoid);

figure(1); clf; 
hold on;
mapshow('mcity/mcity.tiff');
plot(long, lat);
for i=1:length(long)
	plot([long(i) long(i)+kappa(i)*tt(i,1)], [lat(i) lat(i)+kappa(i)*tt(i,2)], 'c')
end
% plot(xr, yr, '*');
figure(2); clf;
plot(s_vec, kappa);
ylim([-0.1 0.1])