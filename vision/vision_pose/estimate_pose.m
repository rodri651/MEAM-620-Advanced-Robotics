function [pos, phi, theta, psi] = estimate_pose(sensor, varargin)
%ESTIMATE_POSE 6DOF pose estimator based on apriltags
%   sensor - struct stored in provided dataset, fields include:
%          - is_ready: logical, indicates whether sensor data is valid
%          - rpy, omg, acc: imu readings (you should not use these in this phase)
%          - img: uint8, 240x376 grayscale image
%          - id: 1xn ids of detected tags, if no tags are present return empty
%                arrays for pos, q
%          - p0, p1, p2, p3, p4: 2xn pixel position of center and
%                                four corners of detected tags
%            Y
%            ^ P3 == P2
%            | || P0 ||
%            | P4 == P1
%            o---------> X
%   varargin - any variables you wish to pass into the function, could be
%              a data structure to represent the map or camera parameters,
%              your decision. But for the purpose of testing, since we don't
%              know what inputs you will use, you have to specify them in
%              init_script by doing
%              estimate_pose_handle = ...
%                  @(sensor) estimate_pose(sensor, your personal input arguments);
%   pos - 3x1 position of the quadrotor in world frame
%   q   - 4x1 quaternion of the quadrotor [w, x, y, z] where q = w + x*i + y*j + z*k
%sensor=data;
id=sensor.id;
if sensor.is_ready==1 && ~isempty(sensor.id)
K=[311.0520 0        201.8724;
 0        311.3885 113.6210;
 0        0        1];
[p0,p1,p2,p3,p4] = get_values(id);
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
H_2=inv(K)*H;
r1 = H_2(:,1)/norm(H_2(:,2));
r2 = H_2(:,2)/norm(H_2(:,2));
r3 = cross(r1,r2);
t = H_2(:,3)/norm(H_2(:,2));
R = [r1,r2,r3];
[U , ~,V]=svd(R);
D = eye(3);
D(3,3) = det(U*V');
R_2 = U*D*V';
T_1 = [R_2,t;0 0 0 1];
T_imu2cam = [cosd(44.5) -sind(44.5) 0 -0.04;-sind(44.5) -cosd(44.5) 0 0;0 0 -1 -0.03;0 0 0 1];
T_f = inv(T_imu2cam)*T_1;
T_v=inv(T_f);
[phi,theta,psi] = RotToRPY_ZXY(T_v(1:3,1:3));
q=RotToQuat(T_v(1:3,1:3));
pos=T_v(1:3,4);
else
    pos=zeros(3,1);
    %q=zeros(4,1);
    phi=0
    theta=0
    psi=0
end

end
