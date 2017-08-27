clear; yalmip('clear');

mptopt('lpsolver', 'gurobi');

syms v p1 p2;

p = [p1 p2];
f = [v, 1/(1+v)];

% Define system s.t. pi = f(i)
A = 0.9*[cos(pi/6)+p1 sin(pi/6); -sin(pi/6) cos(pi/6)+p2];
B = [1; 0];
E = [1; 0];
umax = 0.2;
dmax = 0.02;

% Interval for v
v_ival = [-0.1 0.1];

% perform various tests along the way
sanity_check = 1;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%% Compute systems in convex hull %%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

func_poly = compute_hull(f, v_ival, sanity_check);

% Build polytopes
A_vertices = {};
B_vertices = {};
E_vertices = {};
for vert = func_poly.V'
    A_vertices{end+1} = double(subs(A, p, vert'));
    B_vertices{end+1} = double(subs(B, p, vert'));
    E_vertices{end+1} = double(subs(E, p, vert'));
end

disp(['found ', num2str(length(A_vertices)), ' vertex systems'])

if sanity_check
    % are matrices along curve contained in convex hull?
    AV_cell = cellfun(@vec, A_vertices, 'UniformOutput', false);
    A_poly = Polyhedron('V', [AV_cell{:}]');
    BV_cell = cellfun(@vec, B_vertices, 'UniformOutput', false);
    B_poly = Polyhedron('V', [BV_cell{:}]');
    for val = v_ival(1):range(v_ival)/10:v_ival(2)
        assert(A_poly.contains(vec(double(subs(subs(A, p, f), v, val)))));
        assert(B_poly.contains(vec(double(subs(subs(B, p, f), v, val)))));
    end
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%% Compute invariant set %%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

XUset = Polyhedron('H', [0 0 1 umax; 0 0 -1 umax]);
Dset = Polyhedron('H', [1 dmax; -1 dmax]);
C = Polyhedron('A', [eye(2); -eye(2)], 'b', ones(4,1));

iter = 0;
while not (C <= multi_pre(C, A_vertices, B_vertices, E_vertices, XUset, Dset, 0.0))
  C = intersect(C, multi_pre(C, A_vertices, B_vertices, E_vertices, XUset, Dset, 0.02));
  iter = iter+1;
  disp(['iteration ', num2str(iter), ', ', num2str(size(C.A,1)), ' inequalities'])
end

plot(C)
