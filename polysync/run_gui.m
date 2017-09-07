ENABLE_PARAMETER_ID = uint64(4011);    % parameter ID to enable/disable actuation

parameter_sub = polysync.Subscriber('MessageType', 'ParametersMessage');

parameter_pub = polysync.Publisher('MessageType', 'ParametersMessage');

h = polysync_gui;
handles = guidata(h);

while h.isvalid   % finish at window close
	
	% Parameter message
	msg = PsParametersMessage.zeros();
	[~, msg] = parameter_sub.step();
	if msg.Type == ps_parameter_message_kind.PARAMETER_MESSAGE_RESPONSE
		for i = 1:length(msg.Parameters)
			par = msg.Parameters(i);
			if par.Id == ENABLE_PARAMETER_ID
				if par.Value.U.UllValue
					set(handles.actuation_text, 'String', 'Actuation Enabled');
					set(handles.actuation_text, 'BackgroundColor', 'Green');
				else
					set(handles.actuation_text, 'String', 'Actuation Disabled');
					set(handles.actuation_text, 'BackgroundColor', 'Red');
				end
			end
		end
	end

	% Other messages

	pause(0.2)

end