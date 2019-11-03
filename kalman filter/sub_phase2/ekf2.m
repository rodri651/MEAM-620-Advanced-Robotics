function [X, Z ,state] = ekf2(sensor, varargin)
% EKF2 Extended Kalman Filter with IMU as inputs
%
% INPUTS:
%   sensor - struct stored in provided dataset, fields include
%          - is_ready: logical, indicates whether sensor data is valid
%          - t: sensor timestamp
%          - rpy, omg, acc: imu readings
%          - img: uint8, 240x376 grayscale image
%          - id: 1xn ids of detected tags
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
%              ekf1_handle = ...
%                  @(sensor) ekf2(sensor, your input arguments);
%
% OUTPUTS:
% X - nx1 state of the quadrotor, n should be greater or equal to 9
%     the state should be in the following order
%     [x; y; z; vx; vy; vz; qw; qx; qy; qz; other states you use]
%     we will only take the first 10 rows of X
% OPTIONAL OUTPUTS:
% Z - mx1 measurement of your pose estimator, m shoulb be greater or equal to 6
%     the measurement should be in the following order
%     [x; y; z; qw; qx; qy; qz; other measurement you use]
%     note that this output is optional, it's here in case you want to log your
%     measurement
persistent nut_1 sigmat_1 old_time nu_t sigma_t X_save z
Qt = [1000 1000 200 1 1 1 1 1 1 1 1 1];
Qt = Qt'.*eye(12);

Rt = [1 1 1 0.1 1 1 1 1 0.1];
Rt = Rt'.*eye(9);
% no image, first input, sensor id is not ready
if isempty(nut_1)
    nut_1 = zeros(1,15);
    sigmat_1 = eye(15);
    X = [zeros(2,1);1;zeros(7,1)];
    Z = zeros(7,1);
    old_time = sensor.t;
    state = 1;
    z = 1;
elseif z==1 && ~isempty(sensor.is_ready) && (~isempty(nut_1)) && ~isempty(sensor.id)
    
z = z + 1 ;
[vel, omg] = estimate_vel(sensor);
[pos, quat] = estimate_pose(sensor);
rotm = quat2rotm(quat');
[rot(1) rot(2) rot(3)] = RotToRPY_ZXY(rotm');
nut_1 = [pos',rot,zeros(1,9)];
sigmat_1 = eye(15);
X = [zeros(2,1);1;zeros(7,1)];
Z = zeros(7,1);
old_time = sensor.t;
state = 2;
elseif z==2 && ~isempty(sensor.is_ready) && (~isempty(nut_1)) && ~isempty(sensor.id)


z = z + 1 ;
[vel, omg] = estimate_vel(sensor);
[pos, quat] = estimate_pose(sensor);
rotm = quat2rotm(quat');
[rot(1,1) rot(1,2) rot(1,3)] = RotToRPY_ZXY(rotm');
nut_1 = [pos',rot,vel',zeros(1,6)];
sigmat_1 = eye(15);
X = [nut_1(1:3)';nut_1(7:9)';eul2quat(nut_1(4:6),'XYZ')'];
Z = zeros(7,1);
old_time = sensor.t;
state = 2;
elseif ~isempty(sensor.is_ready) && (~isempty(nut_1)) && ~isempty(sensor.id)
    %prediction step
    dt = sensor.t - old_time;
    u_t = [sensor.acc;sensor.omg];
    noise_eval= zeros(1,12);
    [F,V_t,f] = prediction_input(nut_1,u_t,noise_eval,dt);
    % f,nu_t is 1x9 matrix
    nu_t = nut_1 + dt*f;
    % F, V_t, sigmat_1 is a 9*9 matrix
    sigma_t = F*sigmat_1*F' + V_t*Qt*V_t' ;
    
    
    %update
    Ct = eye(9,15);
    Wt = eye(9,9);
    % Kt is a 9*9 matrix
    Kt = sigma_t*Ct'*inv(Ct*sigma_t*Ct' + Wt*Rt*Wt');
    
    [pos, quat] = estimate_pose(sensor);
    rotm = quat2rotm(quat');
    [rot(1) rot(2) rot(3)] = RotToRPY_ZXY(rotm');
    [vel omg] = estimate_vel(sensor);
    % nut is a 9*1 matrix
    nut = nu_t' + Kt*(eye(9)*[pos;rot';vel] - eye(9)*nu_t(1:9)');
    sigmat = sigma_t - Kt*Ct*sigma_t;
    %nut_1 is a 1*9 matrix
    nut_1 = nut';
    sigmat_1 = sigmat;
    old_time = sensor.t;
    X = [nut(1:3);nut(7:9);eul2quat(nut(4:6)','XYZ')'];
    X_save = X;
    Z = zeros(7,1);
    state = 3;
else
        %prediction step
    Qt = [1 1 1 1 1 1 1 1 1 1 1 1];
    Qt = Qt'.*eye(12);

    Rt = [1 1 1 1 1 1 1 1 1];
    Rt = Rt'.*eye(9);
    dt = sensor.t - old_time;
    u_t = [sensor.acc;sensor.omg];
    noise_eval= zeros(1,12);
    [F,V_t,f] = prediction_input(nut_1,u_t,noise_eval,dt);
    % f,nu_t is 1x9 matrix
    nu_t = nut_1 + dt*f;
    % F, V_t, sigmat_1 is a 9*9 matrix
    sigma_t = F*sigmat_1*F' + V_t*Qt*V_t' ;

    old_time = sensor.t;
    X = [nu_t(1:3)';nu_t(7:9)';eul2quat(nu_t(4:6),'XYZ')'];
    X_save = X;
    Z = zeros(7,1);
    state = 3;
end

end