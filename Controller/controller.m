function [F, M, trpy, drpy] = controller(qd, t, qn, params)
% CONTROLLER quadrotor controller
% The current states are:
%% Inputs:
%
% qd{qn}: state and desired state information for quadrotor #qn (qn
%         will be = 1 since we are only flying a single robot)
%
%  qd{qn}.pos, qd{qn}.vel   position and velocity
%  qd{qn}.euler = [roll;pitch;yaw]
%  qd{qn}.omega     angular velocity in body frame
% 
%  qd{qn}.pos_des, qd{qn}.vel_des, qd{qn}.acc_des  desired position, velocity, accel
%  qd{qn}.yaw_des, qd{qn}.yawdot_des
%
% t: current time
%    
% qn: quadrotor number, should always be 1
%    
% params: various parameters
%  params.I     moment of inertia
%  params.grav  gravitational constant g (9.8...m/s^2)
%  params.mass  mass of robot
%
%% Outputs:
%
% F: total thrust commanded (sum of forces from all rotors)
% M: total torque commanded
% trpy: thrust, roll, pitch, yaw (attitude you want to command!)
% drpy: time derivative of trpy
%
% Using these current and desired states, you have to compute the desired
% controls u, and from there F and M
%
params=crazyflie();
a_d = qd{qn}.acc_des;
v = qd{qn}.vel;
v_d = qd{qn}.vel_des;
p = qd{qn}.pos;
p_d = qd{qn}.pos_des;
g = params.grav;
m = params.mass;
I = params.I;
euler = qd{qn}.euler;
omega = qd{qn}.omega;
yawdot_des = qd{qn}.yawdot_des;
yaw_des = qd{qn}.yaw_des;

% =================== Your code goes here ===================
% ...
kp1=7;      kpt1=2500;
kp2=kp1;      kpt2=kpt1;
kp3=58;    kpt3=20;
kd1=4.3;    kdt1=300;
kd2=kd1;     kdt2=kdt1;
kd3=18;   kdt3=7.55;

% ==============================

% Desired roll, pitch and yaw (in rad).
kdi = [kd1 0 0;0 kd2 0;0 0 kd3];
kpi = [kp1 0 0;0 kp2 0;0 0 kp3];

acc = a_d - kdi*( v - v_d) - kpi*( p - p_d );

phi_des   = (sin(yaw_des)*acc(1)-acc(2)*cos(yaw_des))/g;      %roll 
theta_des = (cos(yaw_des)*acc(1)+acc(2)*sin(yaw_des))/g;          %pitch
psi_des   = yaw_des;

u    = zeros(4,1); % control input u, you should fill this in
u(1) = acc(3)*m + g*m;
% Thrust
F    = u(1);       % This should be F = u(1) from the project handout

% Moment
kpt = [kpt1 0 0;0 kpt2 0;0 0 kpt3]; %phi theta psi
kdt = [kdt1 0 0;0 kdt2 0;0 0 kdt3];


pre = -kpt*angdiff([phi_des;theta_des; psi_des],euler)-kdt*(omega-[0;0;yawdot_des]);
u(2:4) = I*pre;
M    = u(2:4);     % note: params.I has the moment of inertia


% Output trpy and drpy as in hardware
trpy = [F, phi_des, theta_des, psi_des];
drpy = [0, 0,       0,         0];

end


function m = eulzxy2rotmat(ang)
    phi   = ang(1);
    theta = ang(2);
    psi   = ang(3);
    
    m = [[cos(psi)*cos(theta) - sin(phi)*sin(psi)*sin(theta), -cos(phi)*sin(psi), ...
          cos(psi)*sin(theta) + cos(theta)*sin(phi)*sin(psi)];
         [cos(theta)*sin(psi) + cos(psi)*sin(phi)*sin(theta),  cos(phi)*cos(psi), ...
          sin(psi)*sin(theta) - cos(psi)*cos(theta)*sin(phi)];
         [-cos(phi)*sin(theta), sin(phi), cos(phi)*cos(theta)]];
end

function eul = rotmat2eulzxy(R)
    if R(3,2) < 1
        if R(3,2) > -1
            thetaX = asin(R(3,2));
            thetaZ = atan2(-R(1,2), R(2,2));
            thetaY = atan2(-R(3,1), R(3,3));
        else % R(3,2) == -1
            thetaX = -pi/2;
            thetaZ = -atan2(R(1,3),R(1,1));
            thetaY = 0;
        end
    else % R(3,2) == +1
        thetaX = pi/2;
        thetaZ = atan2(R(1,3),R(1,1));
        thetaY = 0;
    end
    eul = [thetaX, thetaY, thetaZ];
end

function w = veemap(R)
    w = [-R(2,3), R(1,3), -R(1,2)];
end
