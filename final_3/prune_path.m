function path_2 = prune_path(path_1)

locs = size(path_1,1);

path_2(1,:) = path_1(1,:);


x = 2;
k = 2;
l = 3;
while l ~= locs && k ~=locs
    diff = (path_1(l,:) - path_1(k,:))/(l-k);
    predir = vecnorm(diff,2,1);
    l = l + 1;
    change = 0;
    while change == 0 && l<locs
        newdiff = (path_1(l,:) - path_1(k,:))/(l-k);
        dir = vecnorm(newdiff,2,1);
        if sum(round(dir,4) == round(predir,4)) == 3
            change = 0;
            l = l + 1;
        else
            change = 1;
            path_2(x,:) = path_1(k,:);
            path_2(x+1,:) = path_1(l-1,:);
            k = l-1;
            l = k + 1;
            x = x + 1;
        end
    end 
end
path_2(end+1,:) = path_1(end,:);
end