function [C_pre] = pre_forall_exists(C, A, B, E, K, XUset, dmax, rho)
	% Compute set
	% {x : ∀ i ∀ di ∃ u,  A{i} x + B{i} u + E{i} di + K{i} + rho B ∈ C}
	% 
	%  (x,u) ∈ XUset
	%  di ∈ [-dmax{i}, dmax{i}]
	%

	if nargin < 8
		rho = 0;
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
	proj = cell(1, 2*N);
	if isempty(dmax)
		% Case without disturbance
		parfor i=1:N
			proj{i} = pre_exists_forall(C, {A{i}}, {B{i}}, [], ...
									  					    {K{i}}, XUset, [], rho);
			proj{i} = myMinHRep(proj{i});
		end
  	C_pre = Polyhedron('H', zeros(0, n+1));
		for i=1:N
	    C_pre = Polyhedron('H', [C_pre.H; proj{i}.H]); % intersect
			C_pre = myMinHRep(C_pre);
		end
	else
		% Case with disturbance
		parfor i=1:2*N
			sys_n = 1 + floor((i-1)/2);
			d = sign(2*mod(i-1,2)-1) * dmax{sys_n};
			proj{i} = pre_exists_forall(C, {A{sys_n}}, {B{sys_n}}, {E{sys_n}}, ...
									   {K{sys_n}}, XUset, d, rho);
			proj{i} = myMinHRep(proj{i});
		end
		C_pre = Polyhedron('H', zeros(0, n+1));
		for i=1:2*N
	    C_pre = Polyhedron('H', [C_pre.H; proj{i}.H]); % intersect
			C_pre = myMinHRep(C_pre);
		end
	end

