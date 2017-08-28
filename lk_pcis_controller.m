 classdef lk_pcis_controller < matlab.System & ...
                         matlab.system.mixin.Propagates & ...
                         matlab.system.mixin.Nondirect
  properties(Nontunable)
    M = 1800;
    lf = 1.2;
    lr = 1.65;
    Caf = 140000;
    Car = 120000;
    Iz = 3270;

    H_x = diag(4);
    f_x = zeros(4);
    H_u = 1;
    f_u = 0;

    sol_opts = struct('DataType', 'double', 'MaxIter', 200, ...
                      'FeasibilityTol', 1e-6, 'IntegrityChecks', true);
  end

  properties(Access = private)
    B_lk;
    E_lk;
    data;
  end
  
  properties(DiscreteState)
    delta_f;
    barrier_val;
  end

  methods
    function obj = lk_pcis_controller(varargin)
      % lk_controller: lane keeping controller based on a barrier QP.
      % Finds inputs s.t. L_f B (x,u) > -gam * B(x) which ensures positivity of B.
      % 
      % Inputs: 
      %  - lkstate: state of lane keeping dynamics [Bus: LKBus]
      %  - accstate: state of acc dynamics [Bus: ACCBus]
      %  - r_d: road curvature
      % 
      % Outputs: 
      %  - delta_f: steering angle [rad]
      % 
      % Car parameters:
      %  - M: car mass [kg]
      %  - lf: distance from center of gravity to front axle [m]
      %  - lr: distance from center of gravity to rear axle [m]
      %  - Caf: cornering stiffness front tires [N/rad]
      %  - Iz: yaw moment of inertia [kg m^2]

      setProperties(obj,nargin,varargin{:});

    end
  end

  methods(Access = protected)
    function setupImpl(obj)
      obj.B_lk = [0; obj.Caf/obj.M; 0; obj.lf*obj.Caf/obj.Iz];
      obj.E_lk = [0; 0; -1; 0];
      data_temp = load('lk_pcis_controller');
      obj.data = data_temp;
      
      obj.delta_f = 0;
      
      obj.barrier_val = 0;
    end
    
    function ds = getDiscreteStateImpl(obj)
        % Return structure of properties with DiscreteState attribute
        ds.delta_f = obj.delta_f;
    end
    
    function [sz,dt,cp] = getDiscreteStateSpecificationImpl(~,~)
        % Return size, data type, and complexity of discrete-state
        % specified in name
        sz = [1 1];
        dt = 'double';
        cp = false;
    end

    % inputs
    function out = getNumInputsImpl(obj)
      out = 1;
    end
    function [o1] = getInputNamesImpl(obj)
      o1 = 'lk_state';
    end
    % outputs
    function out = getNumOutputsImpl(obj)
      out = 2;
    end
    function [n1, n2] = getOutputNamesImpl(obj)
      n1 = 'delta_f';
      n2 = 'control_info';
    end
    function [o1, o2] = getOutputDataTypeImpl(obj)
      o1 = 'double';
      o2 = 'ControlInfoBus';
    end
    function [o1, o2] = getOutputSizeImpl(obj)
      o1 = 1;
      o2 = 1;
    end 
    function [f1, f2] = isOutputFixedSizeImpl(obj)
       f1 = true;
       f2 = true;
    end    
    function [c1, c2] = isOutputComplexImpl(obj)
       c1 = false;
       c2 = false;
    end
    % update
    function updateImpl(obj, lk_state)
      x_lk = [lk_state.y; lk_state.nu; lk_state.dPsi; lk_state.r];

      mu = lk_state.mu;
      r_d = lk_state.r_d;
      
      if mu < 1
          obj.delta_f = 0;
          return
      end
       
      A_lk = [0, 1, mu, 0; 
          0, -(obj.Caf+obj.Car)/obj.M/mu, 0, ((obj.lr*obj.Car-obj.lf*obj.Caf)/obj.M/mu - mu); 
          0, 0, 0, 1;
          0, (obj.lr*obj.Car-obj.lf*obj.Caf)/obj.Iz/mu,  0, -(obj.lf^2 * obj.Caf + obj.lr^2 * obj.Car)/obj.Iz/mu];
      
      dt = obj.data.con.dt;

      A = eye(4) + A_lk * dt + A_lk^2 * dt^2/2 + A_lk^3 * dt^3/3/2;

      A_int = eye(4)*dt + A_lk * dt^2/2 * A_lk^2 * dt^3/3/2 + A_lk^3 * dt^4/4/3/2;

      B = A_int * obj.B_lk;
      E = A_int * obj.E_lk;

      % A = A_lk*obj.data.con.dt + eye(4);
      % B = obj.B_lk*obj.data.con.dt;
      % E = obj.E_lk*obj.data.con.dt;
      K = zeros(4,1);
      
      R_x = obj.H_x;
      r_x = obj.f_x;
      R_u = obj.H_u;
      r_u = obj.f_u;

      A_x = obj.data.poly_A;
      b_x = obj.data.poly_b;

      H = B'*R_x*B + R_u;
      f = r_u + B'*R_x*(A*x_lk + K) + B'*R_x*E*r_d + B'*r_x;
      A_constr = [A_x*B; A_x*B; 1; -1];
      b_constr = [b_x - A_x*A*x_lk - A_x*E*obj.data.con.rd_max; 
                b_x - A_x*A*x_lk - A_x*E*(-obj.data.con.rd_max);
                obj.data.con.df_max;
                obj.data.con.df_max];

      % Objective 0.5 x' H x + f' x
      % 
      L = chol(H, 'lower');
      Linv = L\eye(size(H,1));
  
      [u, status] = mpcqpsolver(Linv, f, -A_constr, -b_constr, ...
                      [], zeros(0,1), ...
                      false(size(A_constr,1),1), obj.sol_opts);
      
      % normalized distance from Polyhedron edge
      scale_vec = [obj.data.con.y_max obj.data.con.nu_max ...
                   obj.data.con.psi_max obj.data.con.r_max];
      Da = A_x.*repmat(scale_vec, size(A_x,1), 1);
      d_list = (b_x - A_x*x_lk)./sum(A_x.*A_x, 2) ...
                .*sqrt(sum(Da.*Da, 2));
      obj.barrier_val = min(d_list);
                  
      if status > 0
        % qp solved successfully
        obj.delta_f = u;
      else
        % infeasible: keep control constant
      end
     
    end

    function [delta_f, control_info] = outputImpl(obj, ~, ~)
        delta_f = obj.delta_f;
        control_info.barrier_val = obj.barrier_val;
        control_info.ddy = 0;
        control_info.qp_status = 0;
    end
    
    function [f1, f2] = isInputDirectFeedthroughImpl(~, ~)
        f1 = false;
        f2 = false;
    end
  end
end
