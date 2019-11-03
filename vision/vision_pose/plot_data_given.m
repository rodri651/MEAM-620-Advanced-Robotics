function []=plot_data_given(vicon, trans,phi, theta,psi)

figure(1)
subplot(1,4,1);
plot(linspace(1,1768,864),trans(1,:));
hold on;
plot(vicon(1,:));
hold on;
xlabel('X');
subplot(1,4,2);
plot(linspace(1,1768,864),trans(2,:));
hold on;
plot(vicon(2,:));
hold on;
xlabel('Y');
subplot(1,4,3)
plot(linspace(1,1768,864),trans(3,:));
hold on;
plot(vicon(3,:));
hold on
xlabel('Z')
subplot(1,4,4)
plot(linspace(1,1768,864),phi(:));
hold on;
plot(vicon(4,:));
hold on
xlabel('phi');
% subplot(2,3,5)
% plot(linspace(1,1768,864),theta(:));
% hold on;
% plot(vicon(1,:));
% hold on
% xlabel('theta')
% subplot(2,3,6)
% plot(linspace(1,1768,864),psi(:));
% hold on;
% plot(vicon(6,:));
% hold on
% xlabel('psi')
end