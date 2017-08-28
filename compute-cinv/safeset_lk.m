% Question: how does discretization impact algo?

clear; yalmip('clear');
global sdpopt

sdpopt = sdpsettings('verbose', 0);
mptopt('lpsolver', 'gurobi');

% define constants
con = constants;

syms v p1 p2;
f = [1/v, v];
p = [p1, p2];

% Define system s.t. pi = f(i)
c22 = -((con.Caf+con.Car)/con.m);
c24 = (con.b*con.Car - con.a*con.Caf)/con.m;
c42 = ((con.b*con.Car-con.a*con.Caf)/con.Iz);
c44 = -((con.a^2*con.Caf+con.b^2*con.Car)/con.Iz);

A = [0 1      p2 0  ;
     0 c22*p1 0  c24*p1-p2 ;
     0 0      0  1  ;
     0 c42*p1 0  c44*p1];
B = [0; con.Caf/con.m; 0; con.a*(con.Caf/con.Iz)];
E = [0; 0; -1; 0];

if con.u_max > con.u_min
    D_max = con.rd_max-(p2-con.u_min)*(con.rd_max-con.rd_min)/(con.u_max-con.u_min);
else
    D_max = con.rd_max;
end

% perform various tests along the way
sanity_check = 1;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%% Compute systems in convex hull %%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

func_poly = compute_hull(f, [con.u_min con.u_max], sanity_check);

% Build polytopes
A_vert = {};
B_vert = {};
E_vert = {};
D_max_vert = {};
for vert = func_poly.V'
    A_vert{end+1} = double(subs(A, p, vert'));
    B_vert{end+1} = double(subs(B, p, vert'));
    E_vert{end+1} = double(subs(E, p, vert'));
    D_max_vert{end+1} = double(subs(D_max, p, vert'));
end

disp(['found ', num2str(length(A_vert)), ' vertex systems'])

if sanity_check
    % are matrices along curve contained in convex hull?
    AV_cell = cellfun(@vec, A_vert, 'UniformOutput', false);
    A_poly = Polyhedron('V', [AV_cell{:}]');
    BV_cell = cellfun(@vec, B_vert, 'UniformOutput', false);
    B_poly = Polyhedron('V', [BV_cell{:}]');
    EV_cell = cellfun(@vec, E_vert, 'UniformOutput', false);
    E_poly = Polyhedron('V', [EV_cell{:}]');
    for val = con.u_min:(con.u_max-con.u_min)/10:con.u_max
        assert(A_poly.contains(vec(double(subs(subs(A, p, f), v, val)))));
        assert(B_poly.contains(vec(double(subs(subs(B, p, f), v, val)))));
        assert(E_poly.contains(vec(double(subs(subs(E, p, f), v, val)))));
    end
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%% Stabilize and discretize  %%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% K = joint_stab(A_vert, B, sanity_check);
K = zeros(1,4);

A_vert_stable = cellfun(@(A) A-B*K, A_vert, 'UniformOutput', 0);

A_vert_d = cellfun(@(A) eye(4) + con.dt*A, A_vert_stable, 'UniformOutput', 0);
B_vert_d = cellfun(@(B) con.dt*B,          B_vert, 'UniformOutput', 0);
E_vert_d = cellfun(@(E) con.dt*E,          E_vert, 'UniformOutput', 0);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%% Compute invariant set %%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

rho = 0.005;

C0 = Polyhedron('A', [eye(4); -eye(4)], ...
			          'b', [con.y_max; con.nu_max; con.psi_max; con.r_max; ...
                      con.y_max; con.nu_max; con.psi_max; con.r_max]);
XUset = Polyhedron('H', [-K 1 con.df_max; K -1 con.df_max]);

rho_ball = Polyhedron('A', [eye(4); -eye(4)], 'b', rho*ones(8,1));

% Initialize
C = Polyhedron('H', [0 0 0 0 1]);
Ct = C0;
iter = 0;
tic;

while not (C-rho_ball <= Ct)
  C = Ct;

  Cpre = pre_forall_exists(C, A_vert_d, B_vert_d, ...
                           E_vert_d, [], XUset, D_max_vert, rho);

  if Cpre.isEmptySet
    disp('returned empty')
    Ct = Cpre;
    break
  end

  Ct = Polyhedron('A', [Cpre.A; C0.A], 'b', [Cpre.b; C0.b]);
  Ct = myMinHRep(Ct);

  cc = Ct.chebyCenter;
  time = toc;

  iter = iter+1;
  disp(['iteration ', num2str(iter), ', ', num2str(size(C.A,1)), ...
        ' ineqs, ball ', num2str(cc.r), ', time ', num2str(time)])
end

if ~isEmptySet(Ct)
  disp('finished computing, checking control invariance...')
  % Check
  Ctt = pre_forall_exists(C, A_vert_d, B_vert_d, ...
                          E_vert_d, [], XUset, D_max_vert, 0.);

  assert(Ct < Ctt);    % if no error Ct is controlled invariant

  poly_A = Ct.A;
  poly_b = Ct.b;

  save('lk_pcis_controller', 'poly_A', 'poly_b', 'con')
end