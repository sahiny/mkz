%% test1_controller: control MKZ over polysync bus
% first time trying to control the car in Mcity
% ysahin
function test1_controller_acc_step_input()
  MU_DES = 25;              % desired forward speed of the car (km/h)
  INPUT_STEP = 0.2;      % desired step-input (+ throttle, - brake)
     
  GAINS_ACC = [0.025 0.0131 0.4286];  % K_p, K_i, K_d for ACC
  GAINS_LK = [0.1 0 0];   % K_p, K_i, K_d for LK
  
  MEM_LENGTH_ACC = 100;
  MEM_LENGTH_LK = 20;

  RTK_SENSOR_ID = 1;    % sensor id of RTK GPS
  STOP_DISTANCE = 20;   % start braking [m]
  ST_RATIO = 12;        % steering ratio of car
  WHEEL_RADIUS = 0.24;  % wheel radius [m]
  STEERING_MAX = 0.78;  % max steering     
  THROTTLE_MAX = 0.28;  % max throttle
  BRAKE_MAX = 0.3;      % maximal braking when stopping
  BRAKE_TIME = 5;       % brake ramp time [s]
  
  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

  pub = polysync.Publisher('MessageType', 'ByteArrayMessage');

  sub_mo = polysync.Subscriber('MessageType', 'PlatformMotionMessage', ...
                               'SensorId', RTK_SENSOR_ID);

  sub_wsr = polysync.Subscriber('MessageType', 'PlatformWheelSpeedReportMessage');

  % Set up systems
  rd = road;
  rd.pathfile = '../mcity/highway.ascii';
  rd.circular = 0;
  rd.setup(struct());

  ACC = acc_pid_controller_test1;
  ACC.mu_des = MU_DES/3.6;
  ACC.K_p = GAINS_ACC(1);
  ACC.K_i = GAINS_ACC(2);
  ACC.K_d = GAINS_ACC(3);
  ACC.max_throttle = THROTTLE_MAX;
  ACC.mem_length = MEM_LENGTH_ACC;
  ACC.throttle_step = INPUT_STEP;
  ACC.setup(struct(), 0.0);

  LK = lk_pid_controller_test1;
  LK.K_p = GAINS_LK(1);
  LK.K_i = GAINS_LK(2);
  LK.K_d = GAINS_LK(3);
  LK.mem_length = MEM_LENGTH_LK;
  LK.max_steering = STEERING_MAX;
  LK.setup(struct(), 0.0);

  % phase 0: shifting to D
  % phase 1a: reach and maintain certain speed
  % phase 1b: apply step throttle input
  % phase 2: stopping
  % phase 3: shifting to P

  % timing
  last_timestamp = 0;

  % Phase variables
  brake_com = 0;    % braking phase
  phase = uint8(0);

  % Save last time
  last_time = embedded.fi(0, 'Signedness', 'Unsigned', ...
                          'FractionLength', 0, ...
                          'WordLength', 64);

  % Current wheel speed
  wheel_sp = single(0);

  % Shift to D
  shift(pub, ps_gear_position_kind.GEAR_POSITION_DRIVE, 0.01);

  phase = phase + 1;

  % Control loop
  while phase < uint8(3)
    % Read data
    [idx1, msg_mo] = sub_mo.step();
    [idx2, msg_wsr] = sub_wsr.step();

    if idx2 > 0
      wheel_sp = WHEEL_RADIUS * ...
             (msg_wsr.FrontLeft + msg_wsr.FrontRight + ...
              msg_wsr.RearLeft + msg_wsr.RearRight)/4;
    end

    if idx1 > 0

      % Take care of timing
      dt = 0;
      if last_time ~= embedded.fi(0, 'Signedness', 'Unsigned', ...
                                  'FractionLength', 0, ...
                                  'WordLength', 64)
        dt = double(msg_mo.Timestamp - last_time)/1e6;
      end
      last_time = msg_mo.Timestamp;

      % Convert data
      rawdata = get_data(msg_mo);

      % Tranform data to model states
      [lk_acc_state, road_left] = rd.step(rawdata);
     

      if road_left < STOP_DISTANCE
        phase = uint8(2);
      end

      % Compute steering
      delta_f = LK.step(lk_acc_state, dt);

      lk_acc_state
      delta_f

      if phase == uint8(1)
        % PID controls forward speed
        throttle_com = ACC.step(lk_acc_state, dt);
        if throttle_com > 0
            pub_msg = get_ba_message([], throttle_com, ST_RATIO*delta_f);
        else 
            pub_msg = get_ba_message(throttle_com, [], ST_RATIO*delta_f);
            if abs(wheel_sp) <  1e-2
                phase = uint(3);
            end
        end

      elseif phase == uint8(2)
        % braking phase
        brake_com = max(brake_com + BRAKE_MAX*dt/BRAKE_TIME, BRAKE_MAX);
        pub_msg = get_ba_message(brake_com, [], ST_RATIO*delta_f);
        if abs(wheel_sp) <  1e-2
          phase = phase + 1;
        end

      else
        pub_msg = get_ba_message();
      end
          
      pub_msg.Header.Timestamp = polysync.GetTimestamp;
      pub.step(pub_msg);    

      polysync.Sleep(0.01);
    end
  end

  shift(pub, ps_gear_position_kind.GEAR_POSITION_PARK, 0.01);
end
