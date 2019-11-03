
iter=size(data)
for k=1:864
sensor=data(k);
K=[311.0520 0        201.8724;
 0        311.3885 113.6210;
 0        0        1];
id=sensor(1).id;
[p0,p1,p2,p3,p4] = get_values(id);
img_p0=round(sensor(1).p0);
img_p1=round(sensor(1).p1);
img_p2=round(sensor(1).p2);
img_p3=round(sensor(1).p3);
img_p4=round(sensor(1).p4);
A=[];
for i =1:length(id)
    
    one_a=zeros(10,9);
    one_a(1,:)=[p0(1,i) p0(2,i) 1 0 0 0 -img_p0(1,i)*p0(1,i) -img_p0(1,i)*p0(2,i) -img_p0(1,i)];
    one_a(2,:)=[0 0 0 p0(1,i) p0(2,i) 1 -img_p0(2,i).*p0(1,i) -img_p0(2,i)*p0(2,i) -img_p0(2,i)];
    one_a(3,:)=[p1(1,i) p1(2,i) 1 0 0 0 -img_p1(1,i)*p1(1,i) -img_p1(1,i)*p1(2,i) -img_p1(1,i)];
    one_a(4,:)=[0 0 0 p1(1,i) p1(2,i) 1 -img_p1(2,i)*p1(1,i) -img_p1(2,i)*p1(2,i) -img_p1(2,i)];
    one_a(5,:)=[p2(1,i) p2(2,i) 1 0 0 0 -img_p2(1,i)*p2(1,i) -img_p2(1,i)*p2(2,i) -img_p2(1,i)];
    one_a(6,:)=[0 0 0 p2(1,i) p2(2,i) 1 -img_p2(2,i)*p2(1,i) -img_p2(2,i)*p2(2,i) -img_p2(2,i)];
    one_a(7,:)=[p3(1,i) p3(2,i) 1 0 0 0 -img_p3(1,i)*p3(1,i) -img_p3(1,i)*p3(2,i) -img_p3(1,i)];
    one_a(8,:)=[0 0 0 p3(1,i) p3(2,i) 1 -img_p3(2,i)*p3(1,i) -img_p3(2,i)*p3(2,i) -img_p3(2,i)];
    one_a(9,:)=[p4(1,i) p4(2,i) 1 0 0 0 -img_p4(1,i)*p4(1,i) -img_p4(1,i)*p4(2,i) -img_p4(1,i)];
    one_a(10,:)=[0 0 0 p4(1,i) p4(2,i) 1 -img_p4(2,i)*p4(1,i) -img_p4(2,i)*p4(2,i) -img_p4(2,i)];
    A=[A;one_a];
end
[~, ~, V1]=svd(A);
H_1 = V1(:,9)/V1(9,9);

H = reshape(H_1,[3,3])';
H_2=inv(K)*H;
r1 = H_2(:,1)/norm(H_2(:,1));
r2 = H_2(:,2)/norm(H_2(:,1));
r3 = cross(r1,r2);
t = H_2(:,3)/norm(H_2(:,1));
R = [r1,r2,r3];
[U , ~,V]=svd(R);
D = eye(3);
D(3,3) = det(U*V');
R_2 = U*D*V';
T_1 = [R_2,t;0 0 0 1];
T_imu2cam = [cosd(45) -cosd(45) 0 -0.04;-cosd(45) -cosd(45) 0 0;0 0 -1 -0.03;0 0 0 1];
T_f = (T_imu2cam)*T_1;
T_v=inv(T_f);
[phi(k),theta(k),psi(k)] = RotToRPY_ZXY(T_v(1:3,1:3)');
q=RotToQuat(T_v(1:3,1:3));
trans(:,k)=T_v(1:3,4);
trans(3,k)=abs(T_v(3,4));

end
plot_data_given(vicon,trans,phi,theta,psi);