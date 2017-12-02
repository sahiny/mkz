 classdef comma_ai_LK < matlab.System & ...
                    matlab.system.mixin.Propagates

  properties
  end

  properties(Nontunable, Access = private)
    py_controller;
  end
  
  methods
    function obj = comma_ai_LK(varargin)
    end
  end

  methods(Access = protected)
    function setupImpl(obj)
      sys = py.importlib.import_module('sys');
      sys.path.append('c:\mkz\carsim\commaai');  % hard-coded..
      py.reload(py.importlib.import_module('py_handle'));
      obj.py_controller = py.py_handle.RunLatController(30.0);
    end
        
    % inputs
    function out = getNumInputsImpl(obj)
      out = 3;
    end
    function [o1, o2, o3] = getInputNamesImpl(obj)
      o1 = 'vEgo';
      o2 = 'thetaEgo';
      o3 = 'observed_path';
    end
    % outputs
    function out = getNumOutputsImpl(obj)
      out = 9;
    end
    function [n1, n2, n3, n4, n5, n6, n7, n8, n9] = getOutputNamesImpl(obj)
      n1 = 'steer';
      n2 = 'satflag';
      n3 = 'd_lookahead';
      n4 = 'y_des';
      n5 = 'angle_steers_des';
      n6 = 'd_poly0';
      n7 = 'd_poly1';
      n8 = 'd_poly2';
      n9 = 'd_poly3';
    end
    function [o1, o2, o3, o4, o5, o6, o7, o8, o9] = getOutputDataTypeImpl(obj)
      o1 = 'double';
      o2 = 'double';
      o3 = 'double';
      o4 = 'double';
      o5 = 'double';
      o6 = 'double';
      o7 = 'double';
      o8 = 'double';
      o9 = 'double';
    end
    function [o1, o2, o3, o4, o5, o6, o7, o8, o9] = getOutputSizeImpl(obj)
      o1 = 1;
      o2 = 1;
      o3 = 1;
      o4 = 1;
      o5 = 1;
      o6 = 1;
      o7 = 1;
      o8 = 1;
      o9 = 1;
    end 
    function [f1, f2, f3, f4, f5, f6, f7, f8, f9] = isOutputFixedSizeImpl(obj)
       f1 = true;
       f2 = true;
       f3 = true;
       f4 = true;
       f5 = true;
       f6 = true;
       f7 = true;
       f8 = true;
       f9 = true;
    end    
    function [c1, c2, c3, c4, c5, c6, c7, c8, c9] = isOutputComplexImpl(obj)
       c1 = false;
       c2 = false;
       c3 = false;
       c4 = false;
       c5 = false;
       c6 = false;
       c7 = false;
       c8 = false;
       c9 = false;
    end
    % update
    function [steer, sat_flag, d_lookahead, y_des, angle_steers_des, d_poly0, d_poly1, d_poly2, d_poly3] = stepImpl(obj, vEgo, thetaEgo, observed_path)
      response = obj.py_controller.update(vEgo, thetaEgo, observed_path);
      steer = response{1};
      sat_flag = response{2};
      d_lookahead = response{3};
      y_des = response{4};
      angle_steers_des = response{5};
      d_poly0= response{6};
      d_poly1= response{7};
      d_poly2= response{8};
      d_poly3= response{9};
    end

    function [f1, f2, f3] = isInputDirectFeedthroughImpl(obj, ~, ~, ~)
        f1 = false;
        f2 = false;
        f3 = false;
    end
  end
end
