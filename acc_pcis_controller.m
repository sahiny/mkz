 classdef acc_pcis_controller < matlab.System & ...
                         matlab.system.mixin.Propagates & ...
                         matlab.system.mixin.Nondirect
  properties(Nontunable)
    M = 1800;
    f0bar = 74.63;
    f1bar = 40.59;
    vl = 26/3.6;

    H_x = diag([1 0.01]);
    f_x = [-28/3.6; 0];
    H_u = 1;
    f_u = 0;

    sol_opts = struct('DataType', 'double', 'MaxIter', 200, ...
                      'FeasibilityTol', 1e-6, 'IntegrityChecks', true);
  end

  properties(Access = private)
    A_acc;
    A_int;
    B_acc;
    E_acc;
    data;
  end
  
  properties(DiscreteState)
    F_w;
    barrier_val;
  end

  methods
    function obj = acc_pcis_controller(varargin)
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
      data_temp = load('acc_pcis_controller', 'poly_A', 'poly_b', 'con');
      obj.data = data_temp;

      A_acc_ct = [obj.f1bar/obj.M 0; -1 0];
      B_acc_ct = [1/obj.M; 0];
      E_acc_ct = [0; 0];

      dt = obj.data.con.dt;

      obj.A_int = eye(2)*dt + A_acc_ct * dt^2/2 * A_acc_ct^2 * dt^3/3/2 + A_acc_ct^3 * dt^4/4/3/2;
      
      obj.A_acc = eye(2) + A_acc_ct * dt + A_acc_ct^2 * dt^2/2 + A_acc_ct^3 * dt^3/3/2;
      obj.B_acc = obj.A_int * B_acc_ct;
      obj.E_acc = obj.A_int * E_acc_ct;

      obj.F_w = 0;
      obj.barrier_val = 0;
    end
    
    function ds = getDiscreteStateImpl(obj)
        % Return structure of properties with DiscreteState attribute
        ds.F_w = obj.F_w;
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
      o1 = 'lk_acc_state';
    end
    % outputs
    function out = getNumOutputsImpl(obj)
      out = 2;
    end
    function [n1, n2] = getOutputNamesImpl(obj)
      n1 = 'F_w';
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
    function updateImpl(obj, state, r_d)
      x_acc = [state.mu; state.h];

      d_lk = state.nu * state.r;
       
      K_ct = [-obj.f0bar/obj.M - d_lk;
              obj.vl];

      A = obj.A_acc;
      B = obj.B_acc;        
      K = obj.A_int * K_ct;
      E = obj.E_acc;
      
      R_x = obj.H_x;
      r_x = obj.f_x;
      R_u = obj.H_u;
      r_u = obj.f_u;

      A_x = obj.data.poly_A;
      b_x = obj.data.poly_b;

      H = B'*R_x*B + R_u;
      f = r_u + B'*R_x*(A*x_acc + K) + B'*r_x;
      A_constr = [A_x*B; A_x*B; 1; -1];
      b_constr = [b_x - A_x*A*x_acc - A_x*K; 
                b_x - A_x*A*x_acc - A_x*K;
                obj.data.con.Fw_max;
                -obj.data.con.Fw_min];

      % Objective 0.5 x' H x + f' x
      % 
      L = chol(H, 'lower');
      Linv = L\eye(size(H,1));
      
      [u, status] = mpcqpsolver(Linv, f, -A_constr, -b_constr, ...
                      [], zeros(0,1), ...
                      false(size(A_constr,1),1), obj.sol_opts);
      
      % distance from Polyhedron edge
      scale_vec = [1 1];
      Da = A_x.*repmat(scale_vec, size(A_x,1), 1);
      d_list = (b_x - A_x*x_acc)./sum(A_x.*A_x, 2) ...
                .*sqrt(sum(Da.*Da, 2));
      obj.barrier_val = min(d_list);
                  
      if status > 0
        % qp solved successfully
        obj.F_w = u;
      else
        % infeasible: accelerate fixed
        obj.F_w = 500;
      end
     
    end

    function [F_w, control_info] = outputImpl(obj, ~, ~)
        F_w = obj.F_w;
        control_info.barrier_val = obj.barrier_val;
        control_info.ddy = 0;
        control_info.qp_status = 0;
    end
    
    function [f1] = isInputDirectFeedthroughImpl(obj, ~)
        f1 = false;
    end
  end
end
