function bus_definitions() 
% BUS_DEFINITIONS initializes a set of bus objects in the MATLAB base workspace 

% Bus object: ControlInfoBus 
clear elems;
elems(1) = Simulink.BusElement;
elems(1).Name = 'barrier_val';
elems(1).Dimensions = 1;
elems(1).DimensionsMode = 'Fixed';
elems(1).DataType = 'double';
elems(1).SampleTime = -1;
elems(1).Complexity = 'real';
elems(1).Min = [];
elems(1).Max = [];
elems(1).DocUnits = '';
elems(1).Description = sprintf('Value of barrier function');

elems(2) = Simulink.BusElement;
elems(2).Name = 'ddy';
elems(2).Dimensions = 1;
elems(2).DimensionsMode = 'Fixed';
elems(2).DataType = 'double';
elems(2).SampleTime = -1;
elems(2).Complexity = 'real';
elems(2).Min = [];
elems(2).Max = [];
elems(2).DocUnits = sprintf('m/s^2');
elems(2).Description = sprintf('lateral acceleration w.r.t road');

elems(3) = Simulink.BusElement;
elems(3).Name = 'qp_status';
elems(3).Dimensions = 1;
elems(3).DimensionsMode = 'Fixed';
elems(3).DataType = 'double';
elems(3).SampleTime = -1;
elems(3).Complexity = 'real';
elems(3).Min = [];
elems(3).Max = [];
elems(3).DocUnits = '';
elems(3).Description = sprintf('exit status from mpcqpsolver');


ControlInfoBus = Simulink.Bus;
ControlInfoBus.HeaderFile = '';
ControlInfoBus.Description = '';
ControlInfoBus.DataScope = 'Auto';
ControlInfoBus.Alignment = -1;
ControlInfoBus.Elements = elems;
clear elems;
assignin('base','ControlInfoBus', ControlInfoBus);

% Bus object: DataBus 
clear elems;
elems(1) = Simulink.BusElement;
elems(1).Name = 'latitude';
elems(1).Dimensions = 1;
elems(1).DimensionsMode = 'Fixed';
elems(1).DataType = 'double';
elems(1).SampleTime = -1;
elems(1).Complexity = 'real';
elems(1).Min = [];
elems(1).Max = [];
elems(1).DocUnits = sprintf('deg');
elems(1).Description = sprintf('latitude');

elems(2) = Simulink.BusElement;
elems(2).Name = 'longitude';
elems(2).Dimensions = 1;
elems(2).DimensionsMode = 'Fixed';
elems(2).DataType = 'double';
elems(2).SampleTime = -1;
elems(2).Complexity = 'real';
elems(2).Min = [];
elems(2).Max = [];
elems(2).DocUnits = sprintf('deg');
elems(2).Description = sprintf('longitude');

elems(3) = Simulink.BusElement;
elems(3).Name = 'el';
elems(3).Dimensions = 1;
elems(3).DimensionsMode = 'Fixed';
elems(3).DataType = 'double';
elems(3).SampleTime = -1;
elems(3).Complexity = 'real';
elems(3).Min = [];
elems(3).Max = [];
elems(3).DocUnits = sprintf('m');
elems(3).Description = sprintf('elevation');

elems(4) = Simulink.BusElement;
elems(4).Name = 'Yaw';
elems(4).Dimensions = 1;
elems(4).DimensionsMode = 'Fixed';
elems(4).DataType = 'double';
elems(4).SampleTime = -1;
elems(4).Complexity = 'real';
elems(4).Min = [];
elems(4).Max = [];
elems(4).DocUnits = sprintf('rad');
elems(4).Description = sprintf('Yaw angle (0 is east)');

elems(5) = Simulink.BusElement;
elems(5).Name = 'YawRate';
elems(5).Dimensions = 1;
elems(5).DimensionsMode = 'Fixed';
elems(5).DataType = 'double';
elems(5).SampleTime = -1;
elems(5).Complexity = 'real';
elems(5).Min = [];
elems(5).Max = [];
elems(5).DocUnits = sprintf('rad/s');
elems(5).Description = sprintf('angular velocity');

elems(6) = Simulink.BusElement;
elems(6).Name = 'Vx';
elems(6).Dimensions = 1;
elems(6).DimensionsMode = 'Fixed';
elems(6).DataType = 'double';
elems(6).SampleTime = -1;
elems(6).Complexity = 'real';
elems(6).Min = [];
elems(6).Max = [];
elems(6).DocUnits = sprintf('m/s');
elems(6).Description = sprintf('inst CG, local x vel');

elems(7) = Simulink.BusElement;
elems(7).Name = 'Vy';
elems(7).Dimensions = 1;
elems(7).DimensionsMode = 'Fixed';
elems(7).DataType = 'double';
elems(7).SampleTime = -1;
elems(7).Complexity = 'real';
elems(7).Min = [];
elems(7).Max = [];
elems(7).DocUnits = sprintf('m/s');
elems(7).Description = sprintf('inst CG, local y vel');

elems(8) = Simulink.BusElement;
elems(8).Name = 'x_gps_cg';
elems(8).Dimensions = 1;
elems(8).DimensionsMode = 'Fixed';
elems(8).DataType = 'double';
elems(8).SampleTime = -1;
elems(8).Complexity = 'real';
elems(8).Min = [];
elems(8).Max = [];
elems(8).DocUnits = sprintf('m');
elems(8).Description = sprintf('distance GPS to CG in local x');

elems(9) = Simulink.BusElement;
elems(9).Name = 'y_gps_cg';
elems(9).Dimensions = 1;
elems(9).DimensionsMode = 'Fixed';
elems(9).DataType = 'double';
elems(9).SampleTime = -1;
elems(9).Complexity = 'real';
elems(9).Min = [];
elems(9).Max = [];
elems(9).DocUnits = sprintf('m');
elems(9).Description = sprintf('distance GPS to CG in local y');

elems(10) = Simulink.BusElement;
elems(10).Name = 'steer_L1';
elems(10).Dimensions = 1;
elems(10).DimensionsMode = 'Fixed';
elems(10).DataType = 'double';
elems(10).SampleTime = -1;
elems(10).Complexity = 'real';
elems(10).Min = [];
elems(10).Max = [];
elems(10).DocUnits = sprintf('rad');
elems(10).Description = sprintf('front left steering angle');

elems(11) = Simulink.BusElement;
elems(11).Name = 'steer_R1';
elems(11).Dimensions = 1;
elems(11).DimensionsMode = 'Fixed';
elems(11).DataType = 'double';
elems(11).SampleTime = -1;
elems(11).Complexity = 'real';
elems(11).Min = [];
elems(11).Max = [];
elems(11).DocUnits = sprintf('rad');
elems(11).Description = sprintf('front right steering angle');

elems(12) = Simulink.BusElement;
elems(12).Name = 'Alpha_L1';
elems(12).Dimensions = 1;
elems(12).DimensionsMode = 'Fixed';
elems(12).DataType = 'double';
elems(12).SampleTime = -1;
elems(12).Complexity = 'real';
elems(12).Min = [];
elems(12).Max = [];
elems(12).DocUnits = sprintf('rad');
elems(12).Description = sprintf('front left slip angle');

elems(13) = Simulink.BusElement;
elems(13).Name = 'Alpha_R1';
elems(13).Dimensions = 1;
elems(13).DimensionsMode = 'Fixed';
elems(13).DataType = 'double';
elems(13).SampleTime = -1;
elems(13).Complexity = 'real';
elems(13).Min = [];
elems(13).Max = [];
elems(13).DocUnits = sprintf('rad');
elems(13).Description = sprintf('front right slip angle');

elems(14) = Simulink.BusElement;
elems(14).Name = 'Fy_L1';
elems(14).Dimensions = 1;
elems(14).DimensionsMode = 'Fixed';
elems(14).DataType = 'double';
elems(14).SampleTime = -1;
elems(14).Complexity = 'real';
elems(14).Min = [];
elems(14).Max = [];
elems(14).DocUnits = sprintf('N');
elems(14).Description = sprintf('front left lateral force');

elems(15) = Simulink.BusElement;
elems(15).Name = 'Fy_R1';
elems(15).Dimensions = 1;
elems(15).DimensionsMode = 'Fixed';
elems(15).DataType = 'double';
elems(15).SampleTime = -1;
elems(15).Complexity = 'real';
elems(15).Min = [];
elems(15).Max = [];
elems(15).DocUnits = sprintf('N');
elems(15).Description = sprintf('front right lateral force');


elems(16) = Simulink.BusElement;
elems(16).Name = 'Fz_L1';
elems(16).Dimensions = 1;
elems(16).DimensionsMode = 'Fixed';
elems(16).DataType = 'double';
elems(16).SampleTime = -1;
elems(16).Complexity = 'real';
elems(16).Min = [];
elems(16).Max = [];
elems(16).DocUnits = sprintf('N');
elems(16).Description = sprintf('front left vertical force');

elems(17) = Simulink.BusElement;
elems(17).Name = 'Fz_R1';
elems(17).Dimensions = 1;
elems(17).DimensionsMode = 'Fixed';
elems(17).DataType = 'double';
elems(17).SampleTime = -1;
elems(17).Complexity = 'real';
elems(17).Min = [];
elems(17).Max = [];
elems(17).DocUnits = sprintf('N');
elems(17).Description = sprintf('front right vertical force');

elems(18) = Simulink.BusElement;
elems(18).Name = 'Fx';
elems(18).Dimensions = 1;
elems(18).DimensionsMode = 'Fixed';
elems(18).DataType = 'double';
elems(18).SampleTime = -1;
elems(18).Complexity = 'real';
elems(18).Min = [];
elems(18).Max = [];
elems(18).DocUnits = sprintf('N');
elems(18).Description = sprintf('longitudinal force');

elems(19) = Simulink.BusElement;
elems(19).Name = 'Gear_CL';
elems(19).Dimensions = 1;
elems(19).DimensionsMode = 'Fixed';
elems(19).DataType = 'double';
elems(19).SampleTime = -1;
elems(19).Complexity = 'real';
elems(19).Min = [];
elems(19).Max = [];
elems(19).DocUnits = sprintf('');
elems(19).Description = sprintf('Gear');

elems(20) = Simulink.BusElement;
elems(20).Name = 'RGear_Tr';
elems(20).Dimensions = 1;
elems(20).DimensionsMode = 'Fixed';
elems(20).DataType = 'double';
elems(20).SampleTime = -1;
elems(20).Complexity = 'real';
elems(20).Min = [];
elems(20).Max = [];
elems(20).DocUnits = sprintf('');
elems(20).Description = sprintf('Gear ratio');


elems(21) = Simulink.BusElement;
elems(21).Name = 'AV_Eng';
elems(21).Dimensions = 1;
elems(21).DimensionsMode = 'Fixed';
elems(21).DataType = 'double';
elems(21).SampleTime = -1;
elems(21).Complexity = 'real';
elems(21).Min = [];
elems(21).Max = [];
elems(21).DocUnits = sprintf('rpm');
elems(21).Description = sprintf('Engine crankshaft spin');

elems(22) = Simulink.BusElement;
elems(22).Name = 'Throttle';
elems(22).Dimensions = 1;
elems(22).DimensionsMode = 'Fixed';
elems(22).DataType = 'double';
elems(22).SampleTime = -1;
elems(22).Complexity = 'real';
elems(22).Min = [];
elems(22).Max = [];
elems(22).DocUnits = sprintf('');
elems(22).Description = sprintf('Normalized throttle at pedal');

elems(23) = Simulink.BusElement;
elems(23).Name = 'Throttle_Eng';
elems(23).Dimensions = 1;
elems(23).DimensionsMode = 'Fixed';
elems(23).DataType = 'double';
elems(23).SampleTime = -1;
elems(23).Complexity = 'real';
elems(23).Min = [];
elems(23).Max = [];
elems(23).DocUnits = sprintf('');
elems(23).Description = sprintf('Normalized throttle at engine');

elems(24) = Simulink.BusElement;
elems(24).Name = 'M_EngOut';
elems(24).Dimensions = 1;
elems(24).DimensionsMode = 'Fixed';
elems(24).DataType = 'double';
elems(24).SampleTime = -1;
elems(24).Complexity = 'real';
elems(24).Min = [];
elems(24).Max = [];
elems(24).DocUnits = sprintf('Nm');
elems(24).Description = sprintf('Engine output torque');

elems(25) = Simulink.BusElement;
elems(25).Name = 'AVy_L1';
elems(25).Dimensions = 1;
elems(25).DimensionsMode = 'Fixed';
elems(25).DataType = 'double';
elems(25).SampleTime = -1;
elems(25).Complexity = 'real';
elems(25).Min = [];
elems(25).Max = [];
elems(25).DocUnits = sprintf('rpm');
elems(25).Description = sprintf('Wheel rpm');

elems(26) = Simulink.BusElement;
elems(26).Name = 'AVy_L2';
elems(26).Dimensions = 1;
elems(26).DimensionsMode = 'Fixed';
elems(26).DataType = 'double';
elems(26).SampleTime = -1;
elems(26).Complexity = 'real';
elems(26).Min = [];
elems(26).Max = [];
elems(26).DocUnits = sprintf('rpm');
elems(26).Description = sprintf('Wheel rpm');

elems(27) = Simulink.BusElement;
elems(27).Name = 'AVy_R1';
elems(27).Dimensions = 1;
elems(27).DimensionsMode = 'Fixed';
elems(27).DataType = 'double';
elems(27).SampleTime = -1;
elems(27).Complexity = 'real';
elems(27).Min = [];
elems(27).Max = [];
elems(27).DocUnits = sprintf('rpm');
elems(27).Description = sprintf('Wheel rpm');

elems(28) = Simulink.BusElement;
elems(28).Name = 'AVy_R2';
elems(28).Dimensions = 1;
elems(28).DimensionsMode = 'Fixed';
elems(28).DataType = 'double';
elems(28).SampleTime = -1;
elems(28).Complexity = 'real';
elems(28).Min = [];
elems(28).Max = [];
elems(28).DocUnits = sprintf('rpm');
elems(28).Description = sprintf('Wheel rpm');

elems(29) = Simulink.BusElement;
elems(29).Name = 'My_Dr_L1';
elems(29).Dimensions = 1;
elems(29).DimensionsMode = 'Fixed';
elems(29).DataType = 'double';
elems(29).SampleTime = -1;
elems(29).Complexity = 'real';
elems(29).Min = [];
elems(29).Max = [];
elems(29).DocUnits = sprintf('N*m');
elems(29).Description = sprintf('Wheel drive moment');

elems(30) = Simulink.BusElement;
elems(30).Name = 'My_Dr_L2';
elems(30).Dimensions = 1;
elems(30).DimensionsMode = 'Fixed';
elems(30).DataType = 'double';
elems(30).SampleTime = -1;
elems(30).Complexity = 'real';
elems(30).Min = [];
elems(30).Max = [];
elems(30).DocUnits = sprintf('N*m');
elems(30).Description = sprintf('Wheel drive moment');

elems(31) = Simulink.BusElement;
elems(31).Name = 'My_Dr_R1';
elems(31).Dimensions = 1;
elems(31).DimensionsMode = 'Fixed';
elems(31).DataType = 'double';
elems(31).SampleTime = -1;
elems(31).Complexity = 'real';
elems(31).Min = [];
elems(31).Max = [];
elems(31).DocUnits = sprintf('N*m');
elems(31).Description = sprintf('Wheel drive moment');

elems(32) = Simulink.BusElement;
elems(32).Name = 'My_Dr_R2';
elems(32).Dimensions = 1;
elems(32).DimensionsMode = 'Fixed';
elems(32).DataType = 'double';
elems(32).SampleTime = -1;
elems(32).Complexity = 'real';
elems(32).Min = [];
elems(32).Max = [];
elems(32).DocUnits = sprintf('N*m');
elems(32).Description = sprintf('Wheel drive moment');

elems(33) = Simulink.BusElement;
elems(33).Name = 'F_pedal';
elems(33).Dimensions = 1;
elems(33).DimensionsMode = 'Fixed';
elems(33).DataType = 'double';
elems(33).SampleTime = -1;
elems(33).Complexity = 'real';
elems(33).Min = [];
elems(33).Max = [];
elems(33).DocUnits = sprintf('N');
elems(33).Description = sprintf('Brake pedal force');

elems(34) = Simulink.BusElement;
elems(34).Name = 'Bk_pedal';
elems(34).Dimensions = 1;
elems(34).DimensionsMode = 'Fixed';
elems(34).DataType = 'double';
elems(34).SampleTime = -1;
elems(34).Complexity = 'real';
elems(34).Min = [];
elems(34).Max = [];
elems(34).DocUnits = sprintf('mm');
elems(34).Description = sprintf('Brake pedal displacement');


DataBus = Simulink.Bus;
DataBus.HeaderFile = '';
DataBus.Description = '';
DataBus.DataScope = 'Auto';
DataBus.Alignment = -1;
DataBus.Elements = elems;
clear elems;
assignin('base','DataBus', DataBus);

% Bus object: LKBus 
clear elems;
elems(1) = Simulink.BusElement;
elems(1).Name = 'y';
elems(1).Dimensions = 1;
elems(1).DimensionsMode = 'Fixed';
elems(1).DataType = 'double';
elems(1).SampleTime = -1;
elems(1).Complexity = 'real';
elems(1).Min = [];
elems(1).Max = [];
elems(1).DocUnits = sprintf('m');
elems(1).Description = sprintf('lateral displacement (road frame)');

elems(2) = Simulink.BusElement;
elems(2).Name = 'dy';
elems(2).Dimensions = 1;
elems(2).DimensionsMode = 'Fixed';
elems(2).DataType = 'double';
elems(2).SampleTime = -1;
elems(2).Complexity = 'real';
elems(2).Min = [];
elems(2).Max = [];
elems(2).DocUnits = sprintf('m/s');
elems(2).Description = sprintf('lateral speed (road frame)');

elems(3) = Simulink.BusElement;
elems(3).Name = 'mu';
elems(3).Dimensions = 1;
elems(3).DimensionsMode = 'Fixed';
elems(3).DataType = 'double';
elems(3).SampleTime = -1;
elems(3).Complexity = 'real';
elems(3).Min = [];
elems(3).Max = [];
elems(3).DocUnits = sprintf('m/s');
elems(3).Description = sprintf('longitudinal speed');

elems(4) = Simulink.BusElement;
elems(4).Name = 'nu';
elems(4).Dimensions = 1;
elems(4).DimensionsMode = 'Fixed';
elems(4).DataType = 'double';
elems(4).SampleTime = -1;
elems(4).Complexity = 'real';
elems(4).Min = [];
elems(4).Max = [];
elems(4).DocUnits = sprintf('m/s');
elems(4).Description = sprintf('lateral speed');

elems(5) = Simulink.BusElement;
elems(5).Name = 'dPsi';
elems(5).Dimensions = 1;
elems(5).DimensionsMode = 'Fixed';
elems(5).DataType = 'double';
elems(5).SampleTime = -1;
elems(5).Complexity = 'real';
elems(5).Min = [];
elems(5).Max = [];
elems(5).DocUnits = sprintf('rad');
elems(5).Description = sprintf('difference between vehicle and road yaw angles');

elems(6) = Simulink.BusElement;
elems(6).Name = 'r';
elems(6).Dimensions = 1;
elems(6).DimensionsMode = 'Fixed';
elems(6).DataType = 'double';
elems(6).SampleTime = -1;
elems(6).Complexity = 'real';
elems(6).Min = [];
elems(6).Max = [];
elems(6).DocUnits = sprintf('rad/s');
elems(6).Description = sprintf('yaw rate (vehicle body)');

elems(7) = Simulink.BusElement;
elems(7).Name = 'h';
elems(7).Dimensions = 1;
elems(7).DimensionsMode = 'Fixed';
elems(7).DataType = 'double';
elems(7).SampleTime = -1;
elems(7).Complexity = 'real';
elems(7).Min = [];
elems(7).Max = [];
elems(7).DocUnits = sprintf('m');
elems(7).Description = sprintf('headway');

elems(8) = Simulink.BusElement;
elems(8).Name = 'r_d';
elems(8).Dimensions = 1;
elems(8).DimensionsMode = 'Fixed';
elems(8).DataType = 'double';
elems(8).SampleTime = -1;
elems(8).Complexity = 'real';
elems(8).Min = [];
elems(8).Max = [];
elems(8).DocUnits = sprintf('rad/s');
elems(8).Description = sprintf('road yaw rate');


LKBus = Simulink.Bus;
LKBus.HeaderFile = '';
LKBus.Description = '';
LKBus.DataScope = 'Auto';
LKBus.Alignment = -1;
LKBus.Elements = elems;
clear elems;
assignin('base','LKACCBus', LKBus);

% Bus object: RoadBus 
clear elems;
elems(1) = Simulink.BusElement;
elems(1).Name = 'car_x';
elems(1).Dimensions = 1;
elems(1).DimensionsMode = 'Fixed';
elems(1).DataType = 'double';
elems(1).SampleTime = -1;
elems(1).Complexity = 'real';
elems(1).Min = [];
elems(1).Max = [];
elems(1).DocUnits = sprintf('m');
elems(1).Description = sprintf('x position of vehicle CG in global coordinates');

elems(2) = Simulink.BusElement;
elems(2).Name = 'car_y';
elems(2).Dimensions = 1;
elems(2).DimensionsMode = 'Fixed';
elems(2).DataType = 'double';
elems(2).SampleTime = -1;
elems(2).Complexity = 'real';
elems(2).Min = [];
elems(2).Max = [];
elems(2).DocUnits = sprintf('m');
elems(2).Description = sprintf('y position of vehicle CG in global coordinates');

elems(3) = Simulink.BusElement;
elems(3).Name = 'road_x';
elems(3).Dimensions = 1;
elems(3).DimensionsMode = 'Fixed';
elems(3).DataType = 'double';
elems(3).SampleTime = -1;
elems(3).Complexity = 'real';
elems(3).Min = [];
elems(3).Max = [];
elems(3).DocUnits = sprintf('m');
elems(3).Description = sprintf('x for point on road closest to vehicle in global coordinates');

elems(4) = Simulink.BusElement;
elems(4).Name = 'road_y';
elems(4).Dimensions = 1;
elems(4).DimensionsMode = 'Fixed';
elems(4).DataType = 'double';
elems(4).SampleTime = -1;
elems(4).Complexity = 'real';
elems(4).Min = [];
elems(4).Max = [];
elems(4).DocUnits = sprintf('m');
elems(4).Description = sprintf('y for point on road closest to vehicle in global coordinates');

elems(5) = Simulink.BusElement;
elems(5).Name = 'd_road_x';
elems(5).Dimensions = 1;
elems(5).DimensionsMode = 'Fixed';
elems(5).DataType = 'double';
elems(5).SampleTime = -1;
elems(5).Complexity = 'real';
elems(5).Min = [];
elems(5).Max = [];
elems(5).DocUnits = sprintf('m/s');
elems(5).Description = sprintf('road tangent vector w.r.t arclength x component');

elems(6) = Simulink.BusElement;
elems(6).Name = 'd_road_y';
elems(6).Dimensions = 1;
elems(6).DimensionsMode = 'Fixed';
elems(6).DataType = 'double';
elems(6).SampleTime = -1;
elems(6).Complexity = 'real';
elems(6).Min = [];
elems(6).Max = [];
elems(6).DocUnits = sprintf('m/s');
elems(6).Description = sprintf('road tangent vector w.r.t arclength y component');

elems(7) = Simulink.BusElement;
elems(7).Name = 'kappa';
elems(7).Dimensions = 1;
elems(7).DimensionsMode = 'Fixed';
elems(7).DataType = 'double';
elems(7).SampleTime = -1;
elems(7).Complexity = 'real';
elems(7).Min = [];
elems(7).Max = [];
elems(7).DocUnits = sprintf('1/m');
elems(7).Description = sprintf('road curvature');

RoadBus = Simulink.Bus;
RoadBus.HeaderFile = '';
RoadBus.Description = '';
RoadBus.DataScope = 'Auto';
RoadBus.Alignment = -1;
RoadBus.Elements = elems;
clear elems;
assignin('base','RoadBus', RoadBus);

