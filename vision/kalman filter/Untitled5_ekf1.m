% storing vicon in a structure
for i = 1:length(time)
    vic(i).t = time(i);
    vic(i).vel = vicon(7:12,i);
end
%time stored
for i = 1:length(data)
    data_t(i) = data(i).t;
end

for i = 1:length(time)
    vicon_t = vic(i);
    if find(data_t == time(i))
        sensor = data(find(data_t == time(i)));
        send(i) = 0;
    else
        sensor = struct('is_ready',[],'t',[],'rpy',[],'omg',[],'acc',[],'img',[],'id',[],'p0',[],'p1',[],'p2',[],'p3',[],'p4',[]);
        send(i) = 1;
    end
        [X(:,i),Z(:,i),state(i)] = ekf1(sensor, vicon_t);
end

qq = quat2rotm(X(4:7,:)');
for i = 1:length(time)
[orientation(1,i) orientation(2,i) orientation(3,i)] = RotToRPY_ZXY(qq(:,:,i));
end
figure(1)
subplot(3,1,1)
plot(X(1,:))
hold on
plot(vicon(1,:))
subplot(3,1,2)
plot(vicon(2,:))
hold on
plot(X(2,:))
subplot(3,1,3)
plot(vicon(3,:))
hold on
plot(X(3,:))
figure(2)
subplot(3,1,1)
plot(vicon(4,:))
hold on
plot(-orientation(3,:))
subplot(3,1,2)
plot(vicon(5,:))
hold on
plot(-orientation(2,:))
subplot(3,1,3)
plot(vicon(6,:))
hold on
plot(-orientation(1,:))


