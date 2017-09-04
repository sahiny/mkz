% Read long/lat from data file and save evenly spaced local waypoints
clear all;

data = load('data/2017-09-01-09-38-48.txt')
lat = data(:,16);
long = data(:,17);
h = 0;

save_file = 'mcity/recorded_path.ascii';	% file for saving path

nom_dist = 1.5;   % distance between waypoints [m]

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

r = road;

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

dlmwrite(save_file, waypoints, 'delimiter', '\t', 'precision', '%.4f')
