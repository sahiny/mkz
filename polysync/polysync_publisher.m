classdef polysync_publisher < matlab.System & matlab.system.mixin.Propagates
    
    properties(Access = protected)
        pub_st;
        pub_br;
        pub_th;
        pub_cm;
    end
    
    methods
        function obj = polysync_publisher(varargin)
        end
    end
    
    methods(Access = protected)
        function setupImpl(obj)
            obj.pub_st = polysync.Publisher('MessageType', 'PlatformSteeringCommandMessage');
            obj.pub_br = polysync.Publisher('MessageType', 'PlatformBrakeCommandMessage');
            obj.pub_th = polysync.Publisher('MessageType', 'PlatformThrottleCommandMessage');
            obj.pub_cm = polysync.Publisher('MessageType', 'CommandMessage');
        end

        function num = getNumInputsImpl(obj)
            num = 5;
        end
        function [o1, o2, o3, o4, o5] = getInputNamesImpl(obj)
            o1 = 'steering';
            o2 = 'throttle';
            o3 = 'brake';
            o4 = 'cinfo_acc';
            o5 = 'cinfo_lk';
        end
        % outputs
        function out = getNumOutputsImpl(obj)
            out = 0;
        end
        
        function [lk_acc_state, road_left] = stepImpl(obj, cmd_st, cmd_th, cmd_br, cinfo_acc, cinfo_lk)
            % Dispatch messages
            msg_st = PsPlatformSteeringCommandMessage.zeros();
            msg_st.Enabled = uint8(1);
            msg_st.SteeringCommandKind = ps_steering_command_kind.STEERING_COMMAND_ANGLE;
            msg_st.SteeringWheelAngle = single(cmd_st);
            msg_st.Timestamp = polysync.GetTimestamp;

            msg_th = PsPlatformThrottleCommandMessage.zeros();
            if (cmd_th > 0)
                msg_th.Enabled = uint8(1);
            end
            msg_th.ThrottleCommandType = ps_throttle_command_kind.THROTTLE_COMMAND_PERCENT;
            msg_th.ThrottleCommand = single(cmd_th);
            msg_th.Timestamp = polysync.GetTimestamp;

            % msg_br = PsPlatformBrakeCommandMessage.zeros();
            % if (cmd_br > 0)
                % msg_br.Enabled = uint8(1);
                % msg_br.BooEnabled = uint8(1); % brake lights
            % end
            % msg_br.BrakeCommandType = ps_brake_command_kind.BRAKE_COMMAND_PERCENT;
            % msg_br.BrakeCommand = single(cmd_br);
            % msg_br.Timestamp = polysync.GetTimestamp;

            msg_cm = PsCommandMessage.zeros();
            msg_cm.Id = embedded.fi(454545, 'Signed', 0, 'WordLength', 64, ...
                                    'FractionLength', 0);
            msg_cm.Data(1).D = ps_parameter_value_kind.PARAMETER_VALUE_DOUBLE;
            msg_cm.Data(1).U.DValue = cinfo_acc.barrier_val;
            msg_cm.Data(2).D = ps_parameter_value_kind.PARAMETER_VALUE_DOUBLE;
            msg_cm.Data(2).U.DValue = cinfo_lk.barrier_val;

            % % Apply controls
            obj.pub_st.step(msg_st);
            obj.pub_th.step(msg_th);
            % obj.pub_br.step(msg_br);
            obj.pub_cm.step(msg_cm);
        end
    end
end