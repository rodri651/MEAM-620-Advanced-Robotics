function [desired_state] = circle(t, qn)
% CIRCLE trajectory generator for a circle
T=12;
% =================== Your code goes here ===================
% You have to set the pos, vel, acc, yaw and yawdot variables
% NOTE: the simulator will spawn the robot to be at the
%       position you return for t == 0
if t<11.8
    pos = [5*cos(2*pi*t/T); 5*sin(2*pi*t/T); 2.5*t/T];
    vel = [-5*2*pi/T*sin(2*pi*t/T); 5*2*pi/T*cos(2*pi*t/T); 2.5/T];
    acc = [-5*4*pi.^2/T.^2*cos(2*pi*t/T); -5*4*pi.^2/T.^2*sin(2*pi*t/T); 0];
    yaw = 0;
    yawdot = 0;
elseif t>11.8 && t<12
        pos = [5*cos(2*pi*t/T); 5*sin(2*pi*t/T); 2.5*t/T];
    vel = [0;0; 0];
    acc = [0; 0; 0];
    yaw = 0;
    yawdot = 0;
 elseif t>12
        pos = [5; 0; 2.5];
    vel = [0;0; 0];
    acc = [0; 0; 0];
    yaw = 0;
    yawdot = 0;   
end
% =================== Your code ends here ===================

desired_state.pos = pos(:);
desired_state.vel = vel(:);
desired_state.acc = acc(:);
desired_state.yaw = yaw;
desired_state.yawdot = yawdot;

end
