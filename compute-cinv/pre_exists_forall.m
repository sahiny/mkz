function [C_pre] = pre_exists_forall(C, A, B, E, K, XUset, Dvert, rho)

	% Compute set
	% {x : ∃ u ∀ i ∀ d, A{i} x + B u + E{i} d + K{i} + rho B ∈ C}
	% 
	%  (x,u) ∈ XUset
	%  d ∈ conv (Dvert)
	%

	if nargin < 7
		rho = 0;
	end

	n = C.Dim;	 		% system dimension
	N = length(A); % number of systems

	Cb = C - Polyhedron('A', [eye(n); -eye(n)], 'b', rho*ones(2*n,1));

	Hx_rep = repmat({Cb.A}, 1, N);
	Hx_diag = blkdiag(Hx_rep{:});

	Hx = Hx_diag * [cell2mat(A') cell2mat(B')];
	hx = repmat(Cb.b, N, 1) - Hx_diag*cell2mat(K') ...
		 - max(Hx_diag*cell2mat(E')*Dvert, [], 2);
	pre_proj = intersect(Polyhedron(Hx, hx), XUset);

	C_pre = projection(pre_proj, 1:n, 'ifourier');