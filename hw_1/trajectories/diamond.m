function [desired_state] = diamond(t, qn)
% DIAMOND trajectory generator for a diamond

% =================== Your code goes here ===================
% You have to set the pos, vel, acc, yaw and yawdot variables
% NOTE: the simulator will spawn the robot to be at the
%       position you return for t == 0
t1=1.8  ;
t1_2=2;%time to comlete one cycle
t2=4.8;
t2_2=5;
t3=7.8;
t3_2=8;
t4=11.8;
t4_2=12
% t1=2  ;
% t1_2=2;%time to comlete one cycle
% t2=4;
% t2_2=5;
% t3=7;
% t3_2=8;
% t4=11;
% t4_2=12
if t<0.001
pos = [0;0;0];
vel = [0; 0; 0];
acc = [0; 0; 0];
yaw = 0;
yawdot = 0;    
elseif t>0.001 && t<t1    
pos = [1/4*t/t1_2; 1.414*t/t1_2; 1.414*t/t1_2];
vel = [1/4/t1_2; 1.414/t1_2; 1.414/t1_2];
acc = [0; 0; 0];
yaw = 0;
yawdot = 0;
elseif t>t1 && t<t1_2    
pos = [1/4*t/t1_2; 1.414*t/t1_2; 1.414*t/t1_2];
vel = [0; 0; 0];
acc = [0; 0; 0];
yaw = 0;
yawdot = 0;
elseif t>=t1_2 && t<t2
pos = [1/4+1/4*(t-t1_2)/(t2_2-t1_2); 1.414-1.414*(t-t1_2)/(t2_2-t1_2); 1.414+1.414*(t-t1_2)/(t2_2-t1_2)];
vel = [1/4*(1)/(t2_2-t1_2); -1.414*(1)/(t2_2-t1_2); 1.414*(1)/(t2_2-t1_2)];
acc = [0; 0; 0];
yaw = 0;
yawdot = 0;
elseif t>t2 && t<t2_2    
pos = [1/4+1/4*(t-t1_2)/(t2_2-t1_2); 1.414-1.414*(t-t1_2)/(t2_2-t1_2); 1.414+1.414*(t-t1_2)/(t2_2-t1_2)];
vel = [0; 0; 0];
acc = [0; 0; 0];
yaw = 0;
yawdot = 0;
elseif t>t2_2 && t<t3
pos = [1/2+1/4*(t-t2_2)/(t3_2-t2_2); 0-1.414*(t-t2_2)/(t3_2-t2_2); 1.414*2-1.414*(t-t2_2)/(t3_2-t2_2)];
vel = [1/4*(1)/(t3_2-t2_2);-1.414*(1)/(t3_2-t2_2);-1.414*(1)/(t3_2-t2_2) ];
acc = [0; 0; 0];
yaw = 0;
yawdot = 0;
elseif t>t3 && t<t3_2    
pos = [1/2+1/4*(t-t2_2)/(t3_2-t2_2); 0-1.414*(t-t2_2)/(t3_2-t2_2); 1.414*2-1.414*(t-t2_2)/(t3_2-t2_2)];
vel = [0; 0; 0];
acc = [0; 0; 0];
yaw = 0;
yawdot = 0;
elseif t>t3_2 && t<t4
pos = [3/4+1/4*(t-t3_2)/(t4_2-t3_2); -1.414+1.414*(t-t3_2)/(t4_2-t3_2); 1.414-1.414*(t-t3_2)/(t4_2-t3_2)];
vel = [1/4*(1)/(t4_2-t3_2); 1.414*(1)/(t4_2-t3_2);-1.414*(1)/(t4_2-t3_2)];
acc = [0; 0; 0];
yaw = 0;
yawdot = 0;
elseif t>t4 && t<t4_2    
pos = [3/4+1/4*(t-t3_2)/(t4_2-t3_2); -1.414+1.414*(t-t3_2)/(t4_2-t3_2); 1.414-1.414*(t-t3_2)/(t4_2-t3_2)];
vel = [0; 0; 0];
acc = [0; 0; 0];
yaw = 0;
yawdot = 0;
elseif t>t4
pos = [1; 0; 0];
vel = [0; 0; 0];
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
