% Read long/lat from data file and save evenly spaced local waypoints
filename = uigetfile;

data = load(filename);

lat = rad2deg(squeeze(data.gpsdata.Latitude.Data));
long = rad2deg(squeeze(data.gpsdata.Longitude.Data));

tidx = and(lat ~= 0, long ~= 0);

lat = lat(tidx);
long = long(tidx);

h = 0;

save_file = [filename, '.ascii'];	% file for saving path

nom_dist = 1;   % distance between waypoints [m]

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

r = road;
r.N_path = 3;
r.N_interp = 10;

% Local coordinates
[xE, yN] = geodetic2enu(lat, long, 0, r.lat0, r.long0, r.h0, wgs84Ellipsoid);

% Extract waypoints that are spaced by at least nom_dist
curr = 1;
waypoints = [xE(curr) yN(curr) 0];

while true
	rel_pos = [xE yN] - waypoints(end, 1:2);
	rel_dist = sqrt(sum(rel_pos.*rel_pos, 2));

	next = find( (curr < (1:length(xE))' ).*( rel_dist > nom_dist ) , 1);

	if isempty(next) || (size(waypoints,1) > 20 && norm(waypoints(2,1:2) - [xE(next) yN(next)]) ...
								    < nom_dist);
		break;
	end

	waypoints(end+1,:) = [xE(next) yN(next) ...
					      waypoints(end,3)+norm([xE(next) yN(next)] - waypoints(end,1:2))];
    curr = next;
end

dlmwrite(save_file, waypoints, 'delimiter', '\t', 'precision', '%.4f');

path_plot(save_file);