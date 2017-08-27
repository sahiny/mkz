%% joint_stab: jointly stabilize a family of linear systems
function [K] = joint_stab(A_mats, B, sanity_check)

	global sdpopt

	if nargin<3
		sanity_check = 0
	end

	% use a feedback to stabilize the system
	P_inv = sdpvar(4,4, 'symmetric');
	K_tilde = sdpvar(1,4, 'full');
	alpha = sdpvar(1,1);
	F = [P_inv >= 1e-5*eye(4)];
	for A = A_mats
		ineq = A{:}*P_inv + P_inv*A{:}' - B*K_tilde - K_tilde'*B' <= -alpha*eye(4);
	    F = [F, ineq];
	end

	optimize(F, -alpha, sdpopt);

	K = value(K_tilde) * inv(value(P_inv));

	if sanity_check
		% check that we stabilized
		for A = A_mats
			assert(all(real(eigs(A{:}-B*K)) < 0))
		end
	end
