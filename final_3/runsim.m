close all;
clear all;
clc;
addpath(genpath('./'));

%% Plan path
disp('Planning ...');
map = load_map('map2.txt', 0.2, 1.0, 0.25);
start = {[19, 2, 2]};
stop = {[2, 18.0, 3.0]};
nquad = length(start);
for qn = 1:nquad
    path{qn} = dijkstra(map, start{qn}, stop{qn}, true);
end
if nquad == 1
    plot_path(map, path{1});
else
    % you could modify your plot_path to handle cell input for multiple robots
end

%% Generate trajectory
disp('Generating Trajectory ...');
trajectory_generator([], [], map, path{1});

%% Run trajectory
trajectory = test_trajectory(start, stop, map, path, true); % with visualization
