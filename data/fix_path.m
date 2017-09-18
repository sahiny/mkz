% Script to manually modify waypoints along path

tiff_file = '../mcity/mcity.tiff';			% background geotiff layer
orig_file = '../mcity/mcity_east.ascii';	% path to work with
save_file = '../mcity/mcity_east.ascii';	% file for saving modified path

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

orig_local = load(orig_file);

clf; hold on

% Show satellite layer
if ~isempty(tiff_file)
	mapshow(tiff_file)
end

r = road;
r.pathfile = orig_file;
r.setup(struct('latitude', 0, 'longitude', 0));

% Convert to GPS
[orig_lat, orig_long] = enu2geodetic(orig_local(:,1), ...
									 orig_local(:,2), 0, ...
									 r.lat0, r.long0, r.h0, ...
									 wgs84Ellipsoid);


% extract curvature information for each waypoint
s_vec = 0:0.25:orig_local(end,end);

[rc, drc, kappa, ~] = arrayfun(@(s) r.get_road_state(s), ...
					  s_vec, ...
					  'UniformOutput', false);
rc = cell2mat(rc);
drc = cell2mat(drc)';
kappa = 0.0001*cell2mat(kappa);   % scaled for visibility

% normal vector to path
tt = [-drc(:,2) drc(:,1)]./sqrt(sum(drc.*drc, 2));
[lat, long, h] = enu2geodetic(rc(1,:), rc(2,:), 0, r.lat0, r.long0, r.h0, wgs84Ellipsoid);

% Plot background layer and curvature markers
plot(orig_long, orig_lat, '*')
for i=1:length(long)
	plot([long(i) long(i)+kappa(i)*tt(i,1)], ...
		 [lat(i) lat(i)+kappa(i)*tt(i,2)], 'c')
end

% Put out impoints
hh = {};
for i=1:size(orig_local,1)
	hh{end+1} = impoint(gca, orig_long(i), orig_lat(i));
end

disp('Drag the points as desired')
pause

% Read point positions
fixed_gps = zeros(size(orig_local,1),2);
for i=1:size(orig_local,1)
	fixed_gps(i,:) = hh{i}.getPosition;
end

% Convert to local coordinates
[xE, yN, zU] = geodetic2enu(fixed_gps(:,2), fixed_gps(:,1), 0, ...
						    r.lat0, r.long0, r.h0, wgs84Ellipsoid);

% Add distance along path
fixed_local = zeros(size(orig_local));
fixed_local(:,1:2) = [xE, yN];
for i=2:size(fixed_local,1)
	fixed_local(i,3) = fixed_local(i-1,3) + ...
		norm(fixed_local(i,1:2) - fixed_local(i-1,1:2));
end

% Write updated path
dlmwrite(save_file, fixed_local, 'delimiter', '\t', 'precision', '%.4f')

