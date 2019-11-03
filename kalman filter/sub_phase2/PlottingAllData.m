addpath('data')

data1 = load('studentdata1.mat');

vicon_in = data1.vicon;
time = data1.time;
sensor_in = data1.data;

vic_struct = struct;

for i = 1:size(time,2)
    vic_struct(i).L = time(i);
    vic_struct(i).vel = vicon_in(7:12,i);
end

for i = 1:size(time,2)
    t_val = vic_struct(i).L;
    t_index = find([sensor_in.t] == t_val);
    vic = vic_struct(i);
    if ~isempty(t_index)
        sensor = sensor_in(t_index);     
    else
        sensor = struct('is_ready',[],'t',[],'rpy',[],'omg',[],'acc',[],'img',[],'id',[],'p0',[],'p1',[],'p2',[],'p3',[],'p4',[]);
    end
    
    [X,Z] = ekf1(sensor,vic);
    
    state_x(i,1) = X(1);
    state_y(i,1) = X(2);
    state_z(i,1) = X(3);
    
    state_q = X(4:7);
    [state_roll(i,1),state_pitch(i,1),state_yaw(i,1)] = RotToRPY_ZXY(QuatToRot(state_q));
end

subplot(3,2,1);
plot(time,state_x,'b');
hold on;
plot(time,vicon_in(1,:),'r');
title('estimated x vs vicon x')

subplot(3,2,3);
plot(time,state_y,'b');
hold on;
plot(time,vicon_in(2,:),'r');
title('estimated y vs vicon y')

subplot(3,2,5);
plot(time,state_z,'b');
hold on;
plot(time,vicon_in(3,:),'r');
title('estimated z vs vicon z')

subplot(3,2,2);
plot(time,-state_roll,'b');
hold on;
plot(time,vicon_in(4,:),'r');
title('estimated roll vs vicon roll')

subplot(3,2,4);
plot(time,-state_pitch,'b');
hold on;
plot(time,vicon_in(5,:),'r');
title('estimated pitch vs vicon pitch')

subplot(3,2,6);
plot(time,-state_yaw,'b');
hold on;
plot(time,vicon_in(6,:),'r');
title('estimated yaw vs vicon yaw')
    