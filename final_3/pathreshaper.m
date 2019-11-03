function path =pathreshaper(path_2,map) 
size_max=size(path_2,1);
i=1;
j=1;
stop=path_2(size_max+1-i,:);
start=path_2(j,:);
% i is index from goal
% j is index from start
path_1=[];
path_3=[];
z=0;
while z~=1%i~=size_max && sum(start==stop)~=3
stop=path_2(size_max+1-i,:);
start=path_2(j,:);
diff_cal=stop-start;
nu_points=max(abs(diff_cal))/min(map.res_xyz);
points=[linspace(start(1),stop(1),nu_points)',linspace(start(2),stop(2),nu_points)',linspace(start(3),stop(3),nu_points)'];
if sum(map.occgrid(pos2ind(map,points)))>0
    if size_max+1-i>j+1  %one point min between i and j
        j=j+1;
    elseif size_max+1-i==j+1  % j is the next point after i
        i=size_max +1 - j;  %make i equal to the next point
        path_1=[path_2(size_max+1-i,:);path_1]; %add i to the path 
        if size_max==i
            z=1
        end
             
    end
else 
    %path_3 is the path from start to final point
    path_3=[path_3;path_2(j,:)];
    i=size_max+1-j;
    if i==size_max  %start is the next point
        z=1;
    end
    j=1;
end
end
path=[path_2(end,:);path_3;path_1];
path=flipud(path);
end