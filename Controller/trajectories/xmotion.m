function [desired_state] = xmotion(t, qn)
% HOVER creates hover

% =================== Your code goes here ===================
% You have to set the pos, vel, acc, yaw and yawdot variables
%    
% NOTE: the simulator will spawn the robot to be at the
%       position you return for t == 0
% total time of simulation T=32

%qcdd=
if t<0.1
pos = [0; 0; 0];
vel = [0; 0; 0];
acc = [0; 0; 0];
yaw = 0;
yawdot = 0;
elseif t>=0.1 && t<3
pos = [0;0;-2];
vel = [0; 0; 0];
acc = [0; 0; 0];
yaw = 0;
yawdot = 0;
else
pos = [0;0;-2];
vel = [0; 0; 0];
acc = [0; 0; 0];
yaw = 0;
yawdot = 0;
end
% elseif t>3.99 && t<5
%    
% pos = [1;0; 0];
% vel = [1.6; 0; 0];
% acc = [0; 0; 0];
% yaw = 0;
% yawdot = 0; 
% else
%  pos = [3.2;0; 0];
% vel = [0; 0; 0];
% acc = [0; 0; 0];
% yaw = 0;
% yawdot = 0;    


% =================== Your code ends here ===================

desired_state.pos = pos(:);
desired_state.vel = vel(:);
desired_state.acc = acc(:);
desired_state.yaw = yaw;
desired_state.yawdot = yawdot;

end

%%
% qs=0   % start position
% qg=1    %end position
% q_dot=17.77     %acceleration
% tb=0.1        %start time
% tg=1            %end time
% if t<tb
% pos=[qs+0.5*q_dot*t^2;0;0]
% vel=[q_dot*t;0;0]
% acc=[q_dot;0;0]
% elseif t<(tg-tb) && t>tb
% pos=[q_s-0.5*q_dot*t_b^2+qdot*t_b*t;0;0]
% vel=[q_dot*tb;0;0]
% accel=[0;0;0]
% elseif t>(tg-tb) && t<tg
% pos=[qg-0.5*q_dot*(tg-t);0;0]
% vel=[q_dot*(tg-t);0;0]
% acc=[-q_dot;0;0]
% end
