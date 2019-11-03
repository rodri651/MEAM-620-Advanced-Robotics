
vel = zeros(3,864);
omg = zeros(3,864);
for i = 1:864
    [vel(:,i), omg(:,i)]=estimate_vel(data(i));
end

plot_data_given_velocity(vicon,vel,omg,1);

