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
A_vertices = {};
B_vertices = {};
E_vertices = {};
D_max_vertices = {};
for vert = func_poly.V'
    A_vertices{end+1} = double(subs(A, p, vert'));
    B_vertices{end+1} = double(subs(B, p, vert'));
    E_vertices{end+1} = double(subs(E, p, vert'));
    D_max_vertices{end+1} = double(subs(D_max, p, vert'));
end

disp(['found ', num2str(length(A_vertices)), ' vertex systems'])

if sanity_check
    % are matrices along curve contained in convex hull?
    AV_cell = cellfun(@vec, A_vertices, 'UniformOutput', false);
    A_poly = Polyhedron('V', [AV_cell{:}]');
    BV_cell = cellfun(@vec, B_vertices, 'UniformOutput', false);
    B_poly = Polyhedron('V', [BV_cell{:}]');
    EV_cell = cellfun(@vec, E_vertices, 'UniformOutput', false);
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

% K = joint_stab(A_vertices, B, sanity_check);
K = zeros(1,4);

A_vertices_stable = cellfun(@(A) A-B*K, A_vertices, 'UniformOutput', 0);

A_vertices_discrete = cellfun(@(A) eye(4) + con.dt*A, A_vertices_stable, 'UniformOutput', 0);
B_vertices_discrete = cellfun(@(B) con.dt*B,          B_vertices, 'UniformOutput', 0);
E_vertices_discrete = cellfun(@(E) con.dt*E,          E_vertices, 'UniformOutput', 0);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%% Compute invariant set %%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

C0 = Polyhedron('A', [eye(4); -eye(4)], ...
			   'b', [con.y_max; con.nu_max; con.psi_max; con.r_max; con.y_max; con.nu_max; con.psi_max; con.r_max]);
XUset = Polyhedron('H', [-K 1 con.df_max; K -1 con.df_max]);

C = C0;
iter = 0;
tic;
while not (C <= multi_pre_diffu(C, A_vertices_discrete, ...
                B_vertices_discrete, E_vertices_discrete, ...
                [], XUset, D_max_vertices, 0.0))
% while true
  Cpre = multi_pre_diffu(C, A_vertices_discrete, B_vertices_discrete, ...
                         E_vertices_discrete, [], XUset, D_max_vertices, 0.005);

  if Cpre.isEmptySet
    disp('returned empty')
    C = Cpre;
    break
  end

  C = Polyhedron('A', [Cpre.A; C0.A], 'b', [Cpre.b; C0.b]);
  C = myMinHRep(C);

  cc = C.chebyCenter;
  time = toc;

  iter = iter+1;
  disp(['iteration ', num2str(iter), ', ', num2str(size(C.A,1)), ...
        ' ineqs, ball ', num2str(cc.r), ', time ', num2str(time)])
end

poly_A = C.A;
poly_b = C.b;

save('lk_pcis_controller', 'poly_A', 'poly_b', 'con')