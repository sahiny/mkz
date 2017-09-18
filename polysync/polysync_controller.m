%% polysync_controller: control MKZ over polysync bus
function polysync_controller()
  RTK_SENSOR_ID = 1;    % sensor id of RTK GPS
  DT = 1e-2;            % time step [s]
  STOP_DISTANCE = 20;   % start braking [m]
  ST_RATIO = 16;  % steering ratio of car

  BRAKE_MAX = 0.3;      % maximal braking when stopping
  BRAKE_TIME = 5;       % brake ramp time [s]

  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

  pub = polysync.Publisher('MessageType', 'ByteArrayMessage');

  sub_mo = polysync.Subscriber('MessageType', 'PlatformMotionMessage', ...
                               'SensorId', RTK_SENSOR_ID);

  % Set up systems
  rd = road;
  rd.pathfile = '../mcity/mcity_east_lower.ascii';
  rd.circular = 0;
  rd.setup(struct());

  ACC = acc_pid_controller;
  ACC.max_throttle = 0.28;
  ACC.setup(struct(), DT);

  LK = lk_pcis_controller;
  LK.H_u = 0.25;   % weight in QP for steering (smaller -> less aggressive centering)
  LK.setup(struct());

  % phase 0: shifting to D
  % phase 1: following route
  % phase 2: stopping
  % phase 3: shifting to P

  % Phase variables
  brake_com = 0;    % braking phase

  phase = uint8(0);
  % Shift to D
  shift(pub, ps_gear_position_kind.GEAR_POSITION_DRIVE, DT);

  phase = phase + 1;

  % Control loop
  while phase < uint8(3)
    % Read data
    [idx, sub_msg] = sub_mo.step();

    if idx > 0
      rawdata = get_data(sub_msg);

      % Tranform data to model states
      [lk_acc_state, road_left] = rd.step(rawdata);

      if road_left < STOP_DISTANCE
        phase = phase + 1;
      end

      % Compute model inputs
      [delta_f, lk_info] = LK(lk_acc_state);

      if phase == uint8(1)
        % ACC controls speed
        [throttle_com] = ACC(lk_acc_state, DT)
        pub_msg = get_ba_message([], throttle_com, ST_RATIO*delta_f);

      elseif phase == uint8(2)
        % Braking phase
        brake_com = max(brake_com + BRAKE_MAX*DT/BRAKE_TIME, BRAKE_MAX);
        pub_msg = get_ba_message(brake_com, [], ST_RATIO*delta_f);

        if abs(rawdata.Vx) <  1e-2
          phase = phase + 1;
        end

      else
        pub_msg = get_ba_message();
      end
          
      pub_msg.Header.Timestamp = polysync.GetTimestamp;
      pub.step(pub_msg);    
      polysync.Sleep(DT);
    end
  end

  shift(pub, ps_gear_position_kind.GEAR_POSITION_PARK, DT);
end
