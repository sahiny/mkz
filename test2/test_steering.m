%% test_turnsignal: test turn signals
function test_steering()
	
    dt = 0.01;
    max_steering = 2.5*pi;

    T = 4;
    k = 2;
    
	pub = polysync.Publisher('MessageType', 'ByteArrayMessage');
    
	for i=0:dt:T
        msg = get_ba_message([],[],i*max_steering/T);
		msg.Header.Timestamp = polysync.GetTimestamp;
		pub.step(msg);
		polysync.Sleep(dt);
    end
        
	for i = 0:dt:T/k
		msg = get_ba_message([],[],max_steering*(1 - k*i/T));
		msg.Header.Timestamp = polysync.GetTimestamp;
		pub.step(msg);
		polysync.Sleep(dt);
    end


    for i = 0:dt:T
		msg = get_ba_message([],[],-max_steering/5);
		msg.Header.Timestamp = polysync.GetTimestamp;
		pub.step(msg);
		polysync.Sleep(dt);
    end
    
    
    for i = 0:dt:T
		msg = get_ba_message([],[],-2*max_steering/5);
		msg.Header.Timestamp = polysync.GetTimestamp;
		pub.step(msg);
		polysync.Sleep(dt);
    end
    
    for i = 0:dt:T
		msg = get_ba_message([],[],0);
		msg.Header.Timestamp = polysync.GetTimestamp;
		pub.step(msg);
		polysync.Sleep(dt);
    end
    
    for i = 0:dt:T
		msg = get_ba_message([],[],-max_steering);
		msg.Header.Timestamp = polysync.GetTimestamp;
		pub.step(msg);
		polysync.Sleep(dt);
    end
    
    for i = 0:dt:T
		msg = get_ba_message([],[],0);
		msg.Header.Timestamp = polysync.GetTimestamp;
		pub.step(msg);
		polysync.Sleep(dt);
    end
