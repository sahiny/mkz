%% multi_pre: backwards reachable set for a family of systems
function [C_pre] = multi_pre(C, A, B, E, K, XUset, Dmax, epsilon)

	%  For (A_i, B_i, E_i), find a set C_pre s.t.
	%  for all x \in C_pre, there exists a u s.t. (x,u) \in XUset
	%  s.t. A_i x + B_i u + E_i d \in C - epsilon for all d \in Dset

	if nargin < 7
		epsilon = 0;
	end

	if isempty(B)
		B = repmat({zeros(C.Dim,0)}, 1, length(A));
		XUset = Polyhedron('H', zeros(0,C.Dim));
	end

	if isempty(K)
		K = repmat({zeros(C.Dim,1)}, 1, length(A));
	end

	n = C.Dim;	   % system dimension
	N = length(A); % number of systems

	% Compute all projections
	projections = cell(1, 2*N);
	for i=1:2*N
		sys_n = 1 + floor((i-1)/2);
		d = sign(2*mod(i-1,2)-1) * Dmax{sys_n};
		projections{i} = multi_pre(C, {A{sys_n}}, {B{sys_n}}, {E{sys_n}}, ...
								   {K{sys_n}}, XUset, d, epsilon);
		projections{i} = myMinHRep(projections{i});
	end

	C_pre = Polyhedron('H', zeros(0, n+1));
	for i=1:2*N
        C_pre = Polyhedron('H', [C_pre.H; projections{i}.H]); % intersect
		C_pre = myMinHRep(C_pre);
	end

