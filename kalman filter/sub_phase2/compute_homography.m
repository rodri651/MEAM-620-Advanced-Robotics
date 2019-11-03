function [H,R_2, t] = compute_homography(sensor) % compute rotation matrix and homography and poistion of bot
id=sensor.id;
if sensor.is_ready==1 && ~isempty(sensor.id)
K=[311.0520 0        201.8724;
 0        311.3885 113.6210;
 0        0        1];
[p0,p1,p2,p3,p4] = get_values_vel(id);
img_p0=(sensor.p0);
img_p1=(sensor.p1);
img_p2=(sensor.p2);
img_p3=(sensor.p3);
img_p4=(sensor.p4);
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
H_2=K\H;
r1 = H_2(:,1)/norm(H_2(:,2));
r2 = H_2(:,2)/norm(H_2(:,2));
r3 = cross(r1,r2);
t = H_2(:,3)/norm(H_2(:,2));
R = [r1,r2,r3];

 [U , ~,V]=svd(R);
 D = eye(3);
 D(3,3) = det(U*V');
 R_2 = U*D*V';

else
    pos=zeros(3,0);
    q=zeros(4,0);
end

end

