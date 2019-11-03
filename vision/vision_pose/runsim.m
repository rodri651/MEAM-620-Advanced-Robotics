i=zeros(1,864)
theta=zeros(1,864)
psi=zeros(1,864)
pos=zeros(3,864)
for i = 1:864
     [pos(:,i) phi(:,i) theta(i) psi(i)]=estimate_pose(data(i));
end
plot_data_given(vicon, pos,-phi, psi,theta)