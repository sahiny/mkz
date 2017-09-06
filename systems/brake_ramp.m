classdef brake_ramp < matlab.System & matlab.system.mixin.Propagates
    
    properties(Nontunable)
        end_ramp = 20;     % maximal brake signal
        end_time = 5;      % length of ramp
    end

    properties(Access = protected)
        ramp_cur;
    end

    methods
        function obj = brake_ramp(varargin)
        end
    end
    
    methods(Access = protected)
        function setupImpl(obj)
            obj.ramp_cur = 0;
        end
        % inputs
        function num = getNumInputsImpl(obj)
            num = 2;
        end
        function [o1, o2] = getInputNamesImpl(obj)
            o1 = 'is_active';
            o2 = 'dt';
        end
        % outputs
        function out = getNumOutputsImpl(obj)
            out = 1;
        end
        function [o1] = getOutputDataTypeImpl(obj)
            o1 = 'double';
        end
        function [o1] = getOutputSizeImpl(obj)
            o1 = 1;
        end
        function [o1] = getOutputNamesImpl(obj)
            o1 = 'braking';
        end
        function [c1] = isOutputComplexImpl(obj)
            c1 = false;
        end
        function [f1] = isOutputFixedSizeImpl(obj)
            f1 = true;
        end
        
        function [braking] = stepImpl(obj, is_active, dt)
            if is_active && obj.ramp_cur < obj.end_ramp
                obj.ramp_cur = obj.ramp_cur + dt * obj.end_ramp / obj.end_time;
            end

            braking = obj.ramp_cur;
        end
    end
end
