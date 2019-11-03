function []=plot_data_given(vicon, V,omg,i)
value = length(V);
figure(i)
subplot(2,3,1);
plot(linspace(1,value*2,value),V(1,:));
hold on;
plot(vicon(7,:));
hold on;
xlabel('tra X');
subplot(2,3,2);
plot(linspace(1,value*2,value),V(2,:));
hold on;
plot(vicon(8,:));
hold on;
xlabel('tra Y');
subplot(2,3,3)
plot(linspace(1,value*2,value),V(3,:));
hold on;
plot(vicon(9,:));
hold on
xlabel('tra Z')
subplot(2,3,4)
plot(linspace(1,value*2,value),omg(1,:));
hold on;
plot(vicon(10,:));
hold on
xlabel('phi');
subplot(2,3,5)
plot(linspace(1,value*2,value),omg(2,:));
hold on;
plot(vicon(11,:));
hold on
xlabel('theta')
subplot(2,3,6)
plot(linspace(1,value*2,value),omg(3,:));
hold on;
plot(vicon(12,:));
hold on
xlabel('psi')

end