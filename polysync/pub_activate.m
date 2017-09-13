%% pub_activate
function pub_activate(publisher, active)

	% Parameter
	param = PsParameter.zeros();
	param.Id = uint32(4011);
	param.Timestamp = polysync.GetTimestamp;
	param.Value.D = ps_parameter_value_kind.PARAMETER_VALUE_ULONGLONG;
	param.Value.U.UllValue = embedded.fi(active, 'Signed', 0, ...
								'WordLength', 64, 'FractionLength', 0);

	% Parameter message (DestGuid = all)
	msg = PsParametersMessage.zeros();
	msg.DestGuid = embedded.fi('Signed', 0, 'WordLength', 64, ...
						'FractionLength', 0, 'hex', 'FFFFFFFF00000000');
	msg.Type = ps_parameter_message_kind.PARAMETER_MESSAGE_SET_VALUE;
	msg.Parameters(1) = param;
	msg.ParametersSize = uint32(1);
	msg.Header.Timestamp = polysync.GetTimestamp;

	publisher.step(msg);

end
