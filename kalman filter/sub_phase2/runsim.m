% storing vicon in a structure

for i = 1:length(data)

        [X(:,i),Z(:,i),state(i)] = ekf2(data(i));
end

data_time = [data.t];
qq = quat2rotm(X(7:10,:)');
for i = 1:length(data)
[orientation(1,i) orientation(2,i) orientation(3,i)] = RotToRPY_ZXY(qq(:,:,i));
end
figure(1)
subplot(3,1,1)
plot(data_time,X(1,:))
xlabel('position X')

hold on
plot(time,vicon(1,:))
legend('Estimated', 'Vicon')
subplot(3,1,2)
plot(time,vicon(2,:))
xlabel('position Y')
hold on
plot(data_time,X(2,:))
legend('Vicon', 'Estimated')
subplot(3,1,3)
plot(time,vicon(3,:))
xlabel('position Z')
hold on
plot(data_time,X(3,:))
legend('Vicon', 'Estimated')
figure(2)
subplot(3,1,1)
plot(time,vicon(6,:))
xlabel('orientation X')
hold on
plot(data_time,-orientation(3,:))
legend('Vicon', 'Estimated')
subplot(3,1,2)
plot(time,vicon(5,:))
xlabel('orientation Y')
hold on
plot(data_time,-orientation(2,:))
legend('Vicon', 'Estimated')
subplot(3,1,3)
plot(time,vicon(4,:))
xlabel('orientation Z')
hold on
plot(data_time,-orientation(1,:))
legend('Vicon', 'Estimated')
figure(3)
subplot(3,1,1)
plot(data_time,X(4,:))
xlabel('velocity X')
hold on
plot(time,vicon(7,:))
legend('Estimated', 'Vicon')
subplot(3,1,2)
plot(time,vicon(8,:))
xlabel('velocity Y')
hold on
plot(data_time,X(5,:))
legend('Vicon', 'Estimated')
subplot(3,1,3)
plot(time,vicon(9,:))
xlabel('velocity Z')
hold on
plot(data_time,X(6,:))
legend('Estimated', 'Vicon')