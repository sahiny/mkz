%% multi_pre: backwards reachable set for a family of systems
function [C_pre] = multi_pre(C, A, B, E, K, XUset, Dset, epsilon)

	%  For (A_i, B_i), find a set C_pre s.t.
	%  exists u in U s.t. for all x \in C_pre, all d \in D, (x,u) \in XUset and
	%  A_i x + B_i u + E_i d \in C - epsilon.

	if nargin < 7
		epsilon = 0;
	end

	n = C.Dim;	 		% system dimension
	N = length(A); % number of systems

	Hx_rep = repmat({C.A}, 1, N);
	Hx_diag = blkdiag(Hx_rep{:});

	Hx = Hx_diag * [cell2mat(A') cell2mat(B')];
	hx = repmat(C.b, N, 1) - Hx_diag*cell2mat(K') ...
		 - max(Hx_diag*cell2mat(E')*Dset, [], 2) - epsilon;
	pre_proj = intersect(Polyhedron(Hx, hx), XUset);

	C_pre = projection(pre_proj, 1:n, 'ifourier');