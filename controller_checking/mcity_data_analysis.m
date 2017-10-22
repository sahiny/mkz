function [results] = mcity_data_analysis( varargin )
	% mcity_data_practice

	%% Process inputs
	experim_list = varargin{1};

	%% Run the desired experiments.
	for n = 1 : length(experim_list)
		experim_num = experim_list(n);
		if experim_num == 1
			mcity_data_practice
		end
		if experim_num == 2
			results{n} = mcity_data_practice2;
		end
end