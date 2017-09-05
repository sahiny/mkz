classdef car_inputs < matlab.System & matlab.system.mixin.Propagates
    
    properties(Nontunable)
        wheel_rad = 0.392;     % wheel radius [m]
        trans_eff = 0.79;      % ratio power at wheels / power at engine
        steer_ratio = 16.5;    % ratio steering wheel angle / wheel angle
        
        % Inverse engine map (engine torque, engine speed) -> throttle
        inv_engine_map_file = 'engine_map.mat';
    end

    properties(Access = protected)
        eng_map_F;
        eng_map_w;
        eng_map_th;
    end

    methods
        function obj = car_inputs(varargin)
        end
    end
    
    methods(Access = protected)
        function setupImpl(obj)
            tt_ = coder.load(obj.inv_engine_map_file);
            obj.eng_map_F = tt_.F_eng_vec;
            obj.eng_map_w = tt_.w_vec;
            obj.eng_map_th = tt_.throttle_mat;
        end
         % Error handling for input values
        function validateInputsImpl(~, ~, ~, in3)
           if  ~isstruct(in3)
                  error(message('simdemos:MLSysBlockMsg:BusInput'));
           end

           if (~(isfield(in3,'AVy_L1') && isfield(in3,'AV_Eng')))
                error(message('simdemos:MLSysBlockMsg:InputBusElements'));
           end
        end
        % inputs
        function num = getNumInputsImpl(obj)
            num = 3;
        end
        function [o1, o2, o3] = getInputNamesImpl(obj)
            o1 = 'delta_f';
            o2 = 'F_w';
            o3 = 'rawdata';
        end
        % outputs
        function out = getNumOutputsImpl(obj)
            out = 2;
        end
        function [o1, o2] = getOutputDataTypeImpl(obj)
            o1 = 'double';
            o2 = 'double';
        end
        function [o1, o2] = getOutputSizeImpl(obj)
            o1 = 1;
            o2 = 1;
        end
        function [o1, o2] = getOutputNamesImpl(obj)
            o1 = 'steering_angle';
            o2 = 'throttle';
        end
        function [c1, c2] = isOutputComplexImpl(obj)
            c1 = false;
            c2 = false;
        end
        function [f1, f2] = isOutputFixedSizeImpl(obj)
            f1 = true;
            f2 = true;
        end
        
        function [steering_angle, throttle] = stepImpl(obj, delta_f, F_w, rawdata)
            avg_rpm = (rawdata.AVy_L1 + rawdata.AVy_L2 + ...
                       rawdata.AVy_R1 + rawdata.AVy_R2) / 4;
            whl_torque = obj.wheel_rad * F_w;

            eng_rpm = rawdata.AV_Eng;

            eng_torque = whl_torque * avg_rpm / eng_rpm / obj.trans_eff;

            c_th = interp2(obj.eng_map_F, obj.eng_map_w, obj.eng_map_th, ...
                               eng_torque,  eng_rpm);

            c_sa = delta_f * obj.steer_ratio;

            % Sometimes get NaNs out from Carsim
            if ~ isnan(c_th)
                throttle = c_th;
            else
                throttle = 0;
            end

            if ~isnan(c_sa)
                steering_angle = c_sa;
            else
                throttle = 0;
            end
        end
    end
end
