mptopt('lpsolver', 'gurobi');

% define constants
con = constants;

syms vi p_i;
f = [vi];
p = [p_i];

A = [con.f1bar/con.m 0; -1 0];
B = [1/con.m; 0];
K = [-con.f0bar/con.m - p_i; con.vl];

% perform various tests along the way
sanity_check = 0;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%% Compute systems in convex hull %%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

func_poly = Polyhedron('V', [con.nu_max*con.r_max; -con.nu_max*con.r_max]);

% Build polytopes
A_vertices = {};
B_vertices = {};
K_vertices = {};
for vert = func_poly.V'
    A_vertices{end+1} = double(subs(A, p, vert'));
    B_vertices{end+1} = double(subs(B, p, vert'));
    K_vertices{end+1} = double(subs(K, p, vert'));
end

disp(['found ', num2str(length(A_vertices)), ' vertex systems'])

if sanity_check
    % are matrices along curve contained in convex hull?
    AV_cell = cellfun(@vec, A_vertices, 'UniformOutput', false);
    A_poly = Polyhedron('V', [AV_cell{:}]');
    BV_cell = cellfun(@vec, B_vertices, 'UniformOutput', false);
    B_poly = Polyhedron('V', [BV_cell{:}]');
    KV_cell = cellfun(@vec, K_vertices, 'UniformOutput', false);
    K_poly = Polyhedron('V', [KV_cell{:}]');
    for val = con.v_ival(1):range(con.v_ival)/10:con.v_ival(2)
        assert(A_poly.contains(vec(double(subs(subs(A, p, f), v, val)))));
        assert(B_poly.contains(vec(double(subs(subs(B, p, f), v, val)))));
        assert(K_poly.contains(vec(double(subs(subs(K, p, f), v, val)))));
    end
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%% Discretize  %%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

A_vert_d = cellfun(@(A) eye(2) + con.dt*A, A_vertices, 'UniformOutput', 0);
B_vert_d = cellfun(@(B) con.dt*B,          B_vertices, 'UniformOutput', 0);
K_vert_d = cellfun(@(K) con.dt*K,          K_vertices, 'UniformOutput', 0);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%% Compute invariant set %%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

C0 = Polyhedron('H', [1 0 con.u_max; -1 0 -con.u_min; 0 -1 -con.h_min]);
XUset = Polyhedron('H', [0 0 1 con.Fw_max; 0 0 -1 -con.Fw_min]);

rho_ball = Polyhedron('A', [eye(2); -eye(2)], 'b', repmat(con.rho_acc,2,1));

% Initialize
C = Polyhedron('H', [0 0 1]);
Ct = C0;
iter = 0;
tic;

while not (C-rho_ball <= Ct)
  C = Ct;

  Cpre = pre_forall_exists(C, A_vert_d, B_vert_d, ...
                           [], K_vert_d, XUset, [], con.rho_acc);

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
  Ctt = pre_forall_exists(Ct, A_vert_d, B_vert_d, ...
                           [], K_vert_d, XUset, [], 0);

  assert(Ct < Ctt);    % if no error Ct is controlled invariant

  C_chain = {Ct};
  for i=1:60
    C_chain{end+1} = pre_forall_exists(C_chain{end}, A_vert_d, B_vert_d, ...
                           [], K_vert_d, XUset, [], 0);
  end

  poly_A = Ct.A;
  poly_b = Ct.b;

  save('acc_pcis_controller', 'poly_A', 'poly_b', 'C_chain', 'con')
end