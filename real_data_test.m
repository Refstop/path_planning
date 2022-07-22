clc;clear all;close all;

%% costmap
map_origin = [-48, -8, 0];
image = imread('hitech6.pgm');
imageNorm = double(image)/255;
imageOccupancy = 1 - imageNorm;
map = occupancyMap(imageOccupancy,20);

setOccupancy(map, -map_origin(1:2), 1);
for i = 1:5
    setOccupancy(map, -[map_origin(1)-0.05*i, map_origin(2)], 1);
    setOccupancy(map, -[map_origin(1), map_origin(2)-0.05*i], 1);
end

% fig1 = figure(1);
% show(map);
% set(fig1, 'OuterPosition', [250, 250, 1500, 1200])

%% robot, lidar data load
laserscan = load("data.txt");
robot_pose = load("pose.txt");
angle_min = -2.05370092392;
angle_max = 2.05370092392;
angle_increment = 0.00581718236208;
angle_n = (angle_max - angle_min)/angle_increment;

% fig2 = figure(2);
% plot(laserscan(:,1), laserscan(:,2),'.');
% grid on;

%% visualize robot's coord
T_om = tf2d(-map_origin);
T_mr = tf2d(robot_pose);
pgm_origin = [0;0;1];
robot_origin_coord = T_om*T_mr*pgm_origin;
robot_origin = robot_origin_coord(1:2,1);
% robot_origin = drawPose(map, -map_origin, robot_pose, false);

laserscan = T_mr*[laserscan';ones(1,length(laserscan))];
laserscan = laserscan(1:2,:)';

% fig3 = figure(3);
% show(map);
% set(fig3, 'OuterPosition', [250, 250, 1500, 1200])

%% extract gridmap's value
sensing_range = 9;
sampling_resolution = 0.01;
ranges = [0:sampling_resolution:sensing_range];

for i = 1:angle_n
    angle = angle_min + (i-1)*angle_increment;
    x(i,:) = ranges*cos(angle);
    y(i,:) = ranges*sin(angle);
end
x = x + robot_origin(1);
y = y + robot_origin(2);

window_size = size(x);
cell_val_threshold = 0.9;
occupied_cell = [];
for i = 1:window_size(1)
    for j = 1:window_size(2)
        cur_cell_val = getOccupancy(map, [x(i,j), y(i,j)]);
        if cur_cell_val > cell_val_threshold
            occupied_cell(:,end+1) = [x(i,j); y(i,j); 1];
        end
    end
end

T_om = tf2d(-map_origin);
temp = inv(T_om)*occupied_cell;
map_cell_pnt = temp(1:2,:)';

% fig4 = figure(4);
% plot(laserscan(:,1), laserscan(:,2),'.');
% hold on; plot(map_cell_pnt(:,1), map_cell_pnt(:,2),'.');
% grid on;
% set(fig4, 'OuterPosition', [250, 250, 2000, 700])

%% dbscan
cluster_threshold = 20;
epsilon = 0.3;
minpts = 5;
idx_ls = dbscan(laserscan,epsilon,minpts);
k_val_ls = unique(idx_ls); % 실제 코드에서는 집합 만들면서 점 개수도 세기
k_ls = length(k_val_ls);
for i = 1:k_ls
    cluster_ls_num(i) = sum(idx_ls==k_val_ls(i));
end
k_val_ls = k_val_ls(cluster_ls_num > cluster_threshold);
k_ls = length(k_val_ls);
ls_t_1 = []; ls_t_2 = [];
ls_1 = laserscan(:,1);
ls_2 = laserscan(:,2);
idx_ls_t = [];
for i = 1:k_ls
    ls_t_1 = [ls_t_1; ls_1(idx_ls==k_val_ls(i))];
    ls_t_2 = [ls_t_2; ls_2(idx_ls==k_val_ls(i))];
    idx_ls_t = [idx_ls_t; idx_ls(idx_ls==k_val_ls(i))];
end
laserscan = [ls_t_1, ls_t_2];
idx_ls = idx_ls_t;

C_ls = [];
for i = 1:k_ls
    if k_val_ls(i) >= 0
        C_ls(end+1,:) = [mean(laserscan(idx_ls==k_val_ls(i),1)), mean(laserscan(idx_ls==k_val_ls(i),2))];
    end
end

idx_map = dbscan(map_cell_pnt,epsilon,minpts);
k_val_map = unique(idx_map);
k_map = length(k_val_map);
for i = 1:k_map
    cluster_map_num(i) = sum(idx_map==k_val_map(i));
end
k_val_map = k_val_map(cluster_map_num > cluster_threshold);
k_map = length(k_val_map);
map_t_1 = []; map_t_2 = [];
map_1 = map_cell_pnt(:,1);
map_2 = map_cell_pnt(:,2);
idx_map_t = [];
for i = 1:k_map
    map_t_1 = [map_t_1; map_1(idx_map==k_val_map(i))];
    map_t_2 = [map_t_2; map_2(idx_map==k_val_map(i))];
    idx_map_t = [idx_map_t; idx_map(idx_map==k_val_map(i))];
end
map_cell_pnt = [map_t_1, map_t_2];
idx_map = idx_map_t;

C_map = [];
for i = 1:k_map
    if k_val_map(i) >= 0
        C_map(end+1,:) = [mean(map_cell_pnt(idx_map==k_val_map(i),1)), mean(map_cell_pnt(idx_map==k_val_map(i),2))];
    end
end

fig5 = figure(5);
% gscatter(map_cell_pnt(:,1),map_cell_pnt(:,2),idx_map);
hold on; gscatter(laserscan(:,1),laserscan(:,2),idx_ls);
hold on; plot(C_ls(:,1), C_ls(:,2), 'o');
hold on; plot(C_map(:,1), C_map(:,2), 'o');
grid on;
set(fig5, 'OuterPosition', [250, 250, 2000, 700])

%% property of cluster by pca(eigenvalue)
% for i = 1:k_ls
%     ls_temp = [laserscan(idx_ls==k_val_ls(i),1), laserscan(idx_ls==k_val_ls(i),2)]
%     C = cov([ls_temp(:,1) - C_ls(i,1), ls_temp(:,2) - C_ls(i,2)]);
%     [E,D] = eig(C);
%     D;
%     hold on; drawArrow(C_ls(i,1)+[0, D(1,1)/2*E(1,1)], C_ls(i,2)+[0, D(1,1)/2*E(2,1)], "red");
%     hold on; drawArrow(C_ls(i,1)+[0, D(2,2)/2*E(1,2)], C_ls(i,2)+[0, D(2,2)/2*E(2,2)], "blue");
% end
i = 3;
ls_temp = [laserscan(idx_ls==k_val_ls(i),1), laserscan(idx_ls==k_val_ls(i),2)];
C = cov([ls_temp(:,1) - C_ls(i,1), ls_temp(:,2) - C_ls(i,2)]);
[E,D] = eig(C);
hold on; drawArrow(C_ls(i,1)+[0, D(1,1)/2*E(1,1)], C_ls(i,2)+[0, D(1,1)/2*E(2,1)], "red");
hold on; drawArrow(C_ls(i,1)+[0, D(2,2)/2*E(1,2)], C_ls(i,2)+[0, D(2,2)/2*E(2,2)], "blue");