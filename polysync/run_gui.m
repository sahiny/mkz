ENABLE_PARAMETER_ID = uint64(4011);    % parameter ID to enable/disable actuation

sub_par = polysync.Subscriber('MessageType', 'ParametersMessage');
sub_com = polysync.Subscriber('MessageType', 'CommandMessage');

pub_par = polysync.Publisher('MessageType', 'ParametersMessage');

h = polysync_gui;
handles = guidata(h);

while h.isvalid   % finish at window close
	
	% Request messages
	param = PsParameter.zeros();
	param.Id = uint32(4011);
	param.Timestamp = polysync.GetTimestamp;

	msg_pub = PsParametersMessage.zeros();
	msg_pub.DestGuid = embedded.fi('Signed', 0, 'WordLength', 64, ...
						'FractionLength', 0, 'hex', 'FFFFFFFF00000000');
	msg_pub.Type = ps_parameter_message_kind.PARAMETER_MESSAGE_GET_VALUE;
	msg_pub.Parameters(1) = param;
	msg_pub.ParametersSize = uint32(1);
	msg_pub.Header.Timestamp = polysync.GetTimestamp;

	pub_par.step(msg_pub);

	pause(0.05)

	% Read parameter messages
	[idx, msg_sub] = sub_par.step();
	if idx > 0 && msg_sub.Type == ps_parameter_message_kind.PARAMETER_MESSAGE_RESPONSE
		for i = 1:length(msg_sub.Parameters)
			par = msg_sub.Parameters(i);
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

	% Read command messages
	[idx, msg_com] = sub_com.step();
	if idx > 0 && msg_com.Id == embedded.fi(454545, 'Signed', 0, 'WordLength', 64, ...
								'FractionLength', 0);

		acc_safe = msg_com.Data(1).U.DValue;
		lk_safe  = msg_com.Data(2).U.DValue;
		set(handles.text_accsafe, 'String', ['ACC barrier: ', num2str(acc_safe,'%1.2f')]);
		set(handles.text_lksafe, 'String', ['LK barrier: ', num2str(lk_safe,'%1.2f')]);
		if lk_safe > 0
			set(handles.text_lksafe, 'BackgroundColor', 'Green')
		else
			set(handles.text_lksafe, 'BackgroundColor', 'Red')
		end
		if acc_safe > 0
			set(handles.text_accsafe, 'BackgroundColor', 'Green')
		else
			set(handles.text_accsafe, 'BackgroundColor', 'Red')
		end

	end

	pause(0.05)

end