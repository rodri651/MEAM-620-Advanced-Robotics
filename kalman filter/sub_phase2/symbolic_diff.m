p = sym('p',[1,3]);
q = sym('q',[1,3]);
bg = sym('bg',[1,3]);
ba = sym('ba',[1,3]);
na = sym('na',[1,3]);
ng = sym('ng',[1,3])
nbg = sym('nbg',[1,3])
nba = sym('nba',[1,3])
syms dt

x_pos_or = sym('x',[1,6]);
x_vel = sym('x_vel',[1,3]);
x = [x_pos_or x_vel bg ba];
u = [p q];
n = [na ng nbg nba];
G = [cos(x(5)) 0 -cos(x(4))*sin(x(5));
    0 1 sin(x(4));
    sin(x(5)) 0 cos(x(4))*cos(x(5))];

we = q - bg - ng;
wwe = (inv(G)*we')'

R = [cos(x(6))*cos(x(5))-sin(x(4))*sin(x(6))*sin(x(5)) -cos(x(4))*sin(x(6)) cos(x(6))*sin(x(5))+cos(x(5))*sin(x(4))*sin(x(6));
    cos(x(5))*sin(x(6))+cos(x(6))*sin(x(4))*sin(x(5)) cos(x(4))*cos(x(6)) sin(x(6))*sin(x(5))-cos(x(6))*cos(x(5))*sin(x(4));
    -cos(x(4))*sin(x(5)) sin(x(4)) cos(x(4))*cos(x(5))]
ae = p - ba - na;
aae = R*ae' + [0;0;9.8];

f = [x_vel wwe aae' nbg nba] 

F_t = eye(15) + jacobian(f , x)*dt

V_t = jacobian(f , n)*dt

%% 
v = sym('v',[9,1])
xm = sym('x',[1,9]);
C = eye(9);
h = C*xm' + v
C_t = jacobian(h,xm)
W_t = jacobian(h,v)
% 
% ut_1 = sym('u_t',[9,1]);
% sigmat_1 = sym('sigmat_1',[9,9])
% %prediction
% q = sym('q',[9,1])
% u_t = ut_1 + dt*f';% f evaluated at ut-1 instead of x,to be used later
% sigma_t = F_t*sigmat_1*F_t' + V_t*(q.*eye(9))*V_t'
% 
% % update 
% R = sym('R',[6,1])
% Kt = sigma_t*C_t'*inv(C_t*sigma_t*C_t' + W_t*(R.*eye(6))*W_t')
% h = C*u_t
% z = C*xm' + v
% ut = u_t + Kt*(z-h)
% sigmat = sigma_t - Kt*C_t*sigma_t
%  