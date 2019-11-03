function [path, num_expanded] = dijkstra(map, start, goal, astar)
% DIJKSTRA Find the shortest path from start to goal.
%   PATH = DIJKSTRA(map, start, goal) returns an mx3 matrix, where each row
%   consists of the (x, y, z) coordinates of a point on the path. The first
%   row is start and the last row is goal. If no path is found, PATH is a
%   0x3 matrix. Consecutive points in PATH should not be farther apart than
%   neighboring voxels in the map (e.g. if 5 consecutive points in PATH are
%   co-linear, don't simplify PATH by removing the 3 intermediate points).
%
%   PATH = DIJKSTRA(map, start, goal, astar) finds the path using euclidean
%   distance to goal as a heuristic if astar is true.
%
%   [PATH, NUM_EXPANDED] = DIJKSTRA(...) returns the path as well as
%   the number of nodes that were expanded while performing the search.

% paramaters:
%   map     - the map object to plan in
%   start   - 1x3 vector of the starting coordinates [x,y,z]
%   goal:   - 1x3 vector of the goal coordinates [x,y,z]
%   astar   - boolean use astar or dijkstra
if nargin < 4
    astar = false;
end

%load on map


% start = [10 4 0];
% goal = [1.25 2 2];

hit=1;
resxy = map.res_xyz(1);
zres =  map.res_xyz(3);
goal_1 = pos2ind(map,goal);
resxy = 1;
startind = pos2ind(map,start);
zres = 1;

jmax=size(map.occgrid,2);
kmax=size(map.occgrid,1);
lmax=size(map.occgrid,3);

% used in neighbour function

%% Struct Declaration
%declaring the start point as a struct
wpts=struct;
wpts(startind).point = startind;
%generate all cells with indices
c=num2cell(1:numel(map.occgrid));
% generate the visited=1 not visited=-1
one = num2cell(-1*ones(length(c),1));
% generate cost to reach as infinity
cosr = num2cell(inf(length(c),1));
% store the position of each point
position = num2cell(ind2pos(map,1:numel(map.occgrid)),2);
[wpts(1:length(c)).point] = c{:};
[wpts(1:length(c)).pos] = one{:};
[wpts(1:length(c)).cost] = cosr{:};
[wpts(1:length(c)).position] = position{:};
wpts(startind).position=start;
wpts(goal_1).position=goal;

%out of the indices find those that are inside the obstacle
%set their pos value to 0
notelegible = find(map.occgrid==1);
elegible=1:numel(map.occgrid);
elegible(notelegible)=[];
zer = num2cell(zeros(length(notelegible),1));
[wpts(notelegible).pos] = zer{:};
% now you have all points, whether elegible or not, cost initialised
%mainarr=[inf(length(elegible),1),elegible'];
%% Start point
% for the starting point
i=startind;
%initial evaluation

if ~isempty(intersect(notelegible,i)) || ~isempty(intersect(notelegible,goal_1))
    path=int16.empty(0,3);
    num_expanded=1;
else
    if isempty(find(notelegible==i,1))
    
        [k j l]=ind2sub([kmax,jmax,lmax],i);
        neigh = [[j k l]+[-1 -1 -1];
            [j k l] + [-1 -1 0];
            [j k l] + [-1 -1 1];
            [j k l] + [-1 0 -1];
            [j k l] + [-1 0 0];
            [j k l] + [-1 0 1];
            [j k l] + [-1 1 -1];
            [j k l] + [-1 1 0];
            [j k l] + [-1 1 1];
            [j k l] + [0 -1 -1];
            [j k l] + [0 -1 0];
            [j k l] + [0 -1 1];
            [j k l] + [0 0 -1];
            [j k l] + [0 0 1];
            [j k l] + [0 1 -1];
            [j k l] + [0 1 0];
            [j k l] + [0 1 1];
            [j k l] + [1 -1 -1];
            [j k l] + [1 -1 0];
            [j k l] + [1 -1 1];
            [j k l] + [1 0 -1];
            [j k l] + [1 0 0];
            [j k l] + [1 0 1];
            [j k l] + [1 1 -1];
            [j k l] + [1 1 0];
            [j k l] + [1 1 1];];
        
        %%
        m=1;
        %neighbour=zeros(6,3);
        neighbour=[];
        for iter = 1:length(neigh)
            if sum((neigh(iter,:)<=[jmax kmax lmax]))==3 && sum((neigh(iter,:)>=[1 1 1]))==3
                neighbour(m,:) = neigh(iter,:);
                m=m+1;
            else
                
            end
        end
        m=1;
        %%
        finneigh=[];
        for iter = 1:size(neighbour,1)
            j=Sub2Ind([kmax,jmax,lmax],neighbour(iter,2), neighbour(iter,1), neighbour(iter,3));
            if map.occgrid(j)==0
                finneigh(m)=j;
                m=m+1;
            end
        end
        
        neigh = finneigh;    %wpts(i).neighbours = intersect(elegible,wpts(i).neighbours);
        wpts(i).cost = 0;
        wpts(i).parents = -1; %(can be used )parent of origin is zero
        wpts(i).pos = 1;  % this element is visited
        %else
        %terminate
    end
    %%
    for j = 1:size(neigh)            %size(wpts(i).neighbours)
        k = neigh(j);%wpts(i).neighbours(j); %check
        cost=wpts(i).cost+ abs(norm([wpts(k).position]-[wpts(i).position]));% + astar*sum(goal-[wpts(i).position]);%sqrt(([wpts(i).position]-goal)*([wpts(i).position]-goal)'); %(((k-i)/resy)==1)*costx-(((i-k)/resy)==1)*costx + ((k-i)/1==1)*costy -((i-k)/1==1)*costy  + (((k-i)/(resx*resy))==1)*costz - (((i-k)/(resx*resy))==1)*costz
        if wpts(k).cost > cost
            wpts(k).cost = cost;
            wpts(k).parents = i;
            mainarr(hit,:)=[cost+ astar*abs(sum(goal-[wpts(i).position])),k];
            hit=hit+1;
        end
    end
    hit=hit-1;

    [val, pos]=min(mainarr(:,1));
    i=mainarr(pos,2);

    %% main loop

    while wpts(i).point~=goal_1 && (val~=inf)

 mainarr(pos,:)=[];
 wpts(i).pos=1;
        

        
        
        [k j l]=ind2sub([kmax,jmax,lmax],i);
        indoo=i;
        neigh = [[j k l]+[-1 -1 -1];
            [j k l] + [-1 -1 0];
            [j k l] + [-1 -1 1];
            [j k l] + [-1 0 -1];
            [j k l] + [-1 0 0];
            [j k l] + [-1 0 1];
            [j k l] + [-1 1 -1];
            [j k l] + [-1 1 0];
            [j k l] + [-1 1 1];
            [j k l] + [0 -1 -1];
            [j k l] + [0 -1 0];
            [j k l] + [0 -1 1];
            [j k l] + [0 0 -1];
            [j k l] + [0 0 1];
            [j k l] + [0 1 -1];
            [j k l] + [0 1 0];
            [j k l] + [0 1 1];
            [j k l] + [1 -1 -1];
            [j k l] + [1 -1 0];
            [j k l] + [1 -1 1];
            [j k l] + [1 0 -1];
            [j k l] + [1 0 0];
            [j k l] + [1 0 1];
            [j k l] + [1 1 -1];
            [j k l] + [1 1 0];
            [j k l] + [1 1 1];];
        
        %%
        m=1;
        %neighbour=zeros(6,3);
        neighbour=[];
        for iter = 1:length(neigh)
            if sum((neigh(iter,:)<=[jmax kmax lmax]))==3 && sum((neigh(iter,:)>=[1 1 1]))==3
                neighbour(m,:) = neigh(iter,:);
                m=m+1;
            else
                
            end
        end
        m=1;
        %%
        finneigh=[];
        for iter = 1:size(neighbour,1)
            j=Sub2Ind([kmax,jmax,lmax],neighbour(iter,2), neighbour(iter,1), neighbour(iter,3));
            if map.occgrid(j)==0
                finneigh(m)=j;
                m=m+1;
            end
        end
        
        neigh = finneigh;    %wpts(i).neighbours = intersect(elegible,wpts(i).neighbours);
        c1=wpts(i).cost;
        ck_pos=[wpts(i).position];
        for j = 1:length(neigh)
            k=neigh(j);
            cost=c1+norm([wpts(k).position]-ck_pos);%+astar*abs(sum(goal-[wpts(i).position])); %(((k-i)/resy)==1)*costx-(((i-k)/resy)==1)*costx + ((k-i)/1==1)*costy -((i-k)/1==1)*costy  + (((k-i)/(resx*resy))==1)*costz - (((i-k)/(resx*resy))==1)*costz;
            if wpts(k).cost > cost
                wpts(k).cost=cost;
                wpts(k).parents = i;
                mainarr(hit,:)=[cost+astar*norm(goal-ck_pos),k];
                hit=hit+1;
            end
        end

        [val, pos]=min(mainarr(:,1));
        i=mainarr(pos,2);

        hit=hit-1;
    end
    if val == inf
        path=int16.empty(0,3);
        num_expanded=size(find([wpts.pos]==-1),2);
    else
        %path=zeros(length(c),1);
        o=1;
        
        while((wpts(i).parents)~=-1)
            
            path(o,:)=wpts(i).position;
            i=wpts(i).parents;
            o=o+1;
        end
        path(o,:)=[wpts(i).position];
        

        mod_path=flipud(path);

        num_expanded=size(find([wpts.pos]==-1),2)
    end
end
%plot_path(map,path);
path=(mod_path);
end
