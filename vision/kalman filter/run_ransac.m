function [V] = run_ransac(z,points,vel)

a_net = zeros(2*length(z),6);
for i = 1 : length(z)
        z_0 = z(i);
        x_0 = points(1,i);
        y_0 = points(2,i);
        a_net(2*i-1:2*i,:)=[-1/z_0 0 x_0/z_0 x_0*y_0 -(1+x_0^2) y_0;
            0 -1/z_0 y_0/z_0 (1+y_0^2) -y_0*x_0 -x_0];
        
               
end

V = a_net\vel(:);

end
