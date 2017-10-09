 classdef lk_barrier_controller < matlab.System & ...
                         matlab.system.mixin.Propagates & ...
                         matlab.system.mixin.Nondirect
  properties(Nontunable)
    gam = 1;
    y_max = 0.9;
    ay_max = 3;
    delta_f_max = 0.5;

    M = 1800;
    lf = 1.2;
    lr = 1.65;
    Caf = 140000;
    Car = 120000;
    Iz = 3270;

    sol_opts = struct('DataType', 'double', 'MaxIter', 200, ...
                      'FeasibilityTol', 1e-6, 'IntegrityChecks', true);
  end

  properties(Access = private)
    B_lk;
    E_lk;
  end

  properties(DiscreteState)
    delta_f;
    barrier_val;
    ddy;
    qp_status; 
  end

  methods
    function obj = lk_controller(varargin)
      % lk_controller: lane keeping controller based on a barrier QP.
      % Finds inputs s.t. L_f B (x,u) > -gam * B(x) which ensures positivity of B.
      % 
      % Inputs: 
      %  - lkstate: state of lane keeping dynamics [Bus: LKBus]
      % 
      % Outputs: 
      %  - delta_f: steering angle [rad]
      %  - barrier_val: value of barrier function [-]
      % 
      % Controller parameters:
      %  - gam: 
      %  - y_max: maximal deviation from trajectory (lane center) [m]
      %  - ay_max: maximal lateral acceleration [m/s^2]
      %  - delta_f_max: maximal absolute steering angle [rad]
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
      obj.delta_f = 0;
      obj.qp_status = 0;
      obj.ddy = 0;
      obj.barrier_val = 0;
    end

    function ds = getDiscreteStateImpl(obj)
        % Return structure of properties with DiscreteState attribute
        ds.delta_f = obj.delta_f;
        ds.barrier_val = obj.barrier_val;
        ds.ddy = obj.ddy;
        ds.qp_status = obj.qp_status;
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
      out = 2;
    end
    function [o1, o2] = getInputNamesImpl(obj)
      o1 = 'lk_state';
      o2 = 'r_d';
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
    function updateImpl(obj, lk_state, r_d)
      x = [lk_state.y; lk_state.nu; lk_state.dPsi; lk_state.r];

      mu = lk_state.mu;
       
      A_lk = [0, 1, mu, 0; 
          0, -(obj.Caf+obj.Car)/obj.M/mu, 0, ((obj.lr*obj.Car-obj.lf*obj.Caf)/obj.M/mu - mu); 
          0, 0, 0, 1;
          0, (obj.lr*obj.Car-obj.lf*obj.Caf)/obj.Iz/mu,  0, -(obj.lf^2 * obj.Caf + obj.lr^2 * obj.Car)/obj.Iz/mu];
      
      y = [1 0 0 0] * x;
      dy = [0 1 mu 0] * x;

      % Barrier condition
      dB = [-sign(dy) -dy/obj.ay_max 0 0];   % grad B = dB * \dot x
      bval = obj.y_max - sign(dy) * y - 0.5*dy^2/obj.ay_max;
      A1 = -dB*obj.B_lk;
      b1 = obj.gam * bval + dB * (A_lk * x + obj.E_lk * r_d);

      % bound on \ddot y 
      c = [0 1 mu 0];   % \ddot y = c \dot x
      A2 = [c*obj.B_lk;
            -c*obj.B_lk];
      b2 = [obj.ay_max - c*(A_lk * x + obj.E_lk * r_d);
            obj.ay_max + c*(A_lk * x + obj.E_lk * r_d)];

      % bound on delta_f
      A3 = [1; -1];
      b3 = [obj.delta_f_max; obj.delta_f_max];

      % Stack it
      A_leq_u = [A1; A2; A3];
      b_leq_u = [b1; b2; b3];

      % Objective 0.5 x' H x + f' x
      % 
      %      L = chol(H, 'lower')
      %      Linv = L\eye(size(H,1))
      %
      Linv = 1;
      f = 0; %-dB * obj.B_lk;

      [u, status] = mpcqpsolver(Linv, f, -A_leq_u, -b_leq_u, ...
                      [], zeros(0,1), ...
                      false(size(A_leq_u,1),1), obj.sol_opts);
 
      if status ~= -1
        obj.delta_f = u;
      end

      dx = A_lk * x + obj.B_lk * obj.delta_f + obj.E_lk * r_d;
      
      obj.barrier_val = bval;
      obj.ddy = [0 1 mu 0] * dx;
      obj.qp_status = status;

    end

    function [delta_f, control_info] = outputImpl(obj, ~, ~)
        delta_f = obj.delta_f;
        control_info.barrier_val = obj.barrier_val;
        control_info.ddy = obj.ddy;
        control_info.qp_status = obj.qp_status;
    end
    
    function [f1, f2] = isInputDirectFeedthroughImpl(obj, ~, ~)
        f1 = false;
        f2 = false;
    end
  end
end
