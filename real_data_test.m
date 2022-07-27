clc;clear all;close all;

%% costmap
map_origin = [-48, -8, 0];
image = imread('hitech6.pgm');
imageNorm = double(image)/255;
imageOccupancy = 1 - imageNorm;
map = occupancyMap(imageOccupancy,20);

% setOccupancy(map, -map_origin(1:2), 1);
% for i = 1:5
%     setOccupancy(map, -[map_origin(1)-0.05*i, map_origin(2)], 1);
%     setOccupancy(map, -[map_origin(1), map_origin(2)-0.05*i], 1);
% end

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
laserscan = laserscan(1:2,:)'; % map 기준 좌표

fig3 = figure(3);
plot(laserscan(:,1), laserscan(:,2), '.')
set(fig3, 'OuterPosition', [250, 250, 2000, 600])
grid on;

%% 1. laserscan만 클러스터링
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

for i = 1:k_ls
    range = [];
    for j = 1:length(idx_ls)
        if idx_ls(j) == k_val_ls(i)
            range(end+1) = norm(C_ls(i,:) - laserscan(j,:));
        end
    end
    r_max(i) = max(range);
end

%% visualize area
for i = 1:k_ls
    for j = 1:360
        areax(j) = C_ls(i,1) + r_max(i)*cosd(j);
        areay(j) = C_ls(i,2) + r_max(i)*sind(j);
    end
%     hold on; plot(areax, areay);
end

%% 2. gridmap과 일정 부분 이상 겹칠 경우 제거
sampling_resolution = 0.03;
cell_val_threshold = 0.9;

culled_laserscan = laserscan;
culled_idx_ls = idx_ls;
culled_k_val_ls = k_val_ls;
culled_C_ls = C_ls;
culling_index = [];

for i = 1:k_ls
    ranges = [0:sampling_resolution:r_max(i)];
    x = []; y = [];
    for j = 1:angle_n
        angle = angle_min + (j-1)*angle_increment;
        x(j,:) = ranges*cos(angle);
        y(j,:) = ranges*sin(angle);
    end

    C_ls_ogm = T_om*[C_ls(i,1); C_ls(i,2); 1];
    x = x + C_ls_ogm(1);
    y = y + C_ls_ogm(2);

    window_size = size(x);
    
    occupied_cell = [];
    for j = 1:window_size(1)
        for k = 1:window_size(2)
            cur_cell_val = getOccupancy(map, [x(j,k), y(j,k)]);
            if cur_cell_val > cell_val_threshold
                occupied_cell(:,end+1) = [x(j,k); y(j,k); 1];
            end
        end
    end
    if length(occupied_cell) > 20
        culled_laserscan(culled_idx_ls == k_val_ls(i),:) = [];
        culled_idx_ls(culled_idx_ls == k_val_ls(i)) = [];
        culling_index(end+1) = i;
    end
    % 잘 안되면 empty가 아니면 코사인 유사도 검사도 해볼것
end
culled_k_val_ls(culling_index) = [];
culled_k_ls = length(culled_k_val_ls);
culled_C_ls(culling_index,:) = [];
r_max(culling_index) = [];
hold on; plot(culled_laserscan(:,1), culled_laserscan(:,2), '.');

%% current robot & goal
r_obs = 0.2;
goal_pose = [6, 1, 0.0412];

robot = triangle(robot_pose(1), robot_pose(2), robot_pose(3));
robot_pgon = polyshape(robot(1,:), robot(2,:));
hold on; plot(robot_pgon);
hold on; plot(robot(1,1), robot(2,1), '*');

goal = triangle(goal_pose(1), goal_pose(2), goal_pose(3));
goal_pgon = polyshape(goal(1,:), goal(2,:));
hold on; plot(goal_pgon);
hold on; plot(goal(1,1), goal(2,1), '*');

%% global path(point to point)
path_len = 50;
global_path_x = linspace(robot_pose(1), goal_pose(1), path_len);
global_path_y = linspace(robot_pose(2), goal_pose(2), path_len);
global_path = [global_path_x; global_path_y];
hold on; plot(global_path_x, global_path_y, 'Color', 'red');

%% collision region of robot
r_robot = 0.4;
for i = 1:360
    robotxr(i) = robot_pose(1) + r_robot*cosd(i);
    robotyr(i) = robot_pose(2) + r_robot*sind(i);
end
hold on; plot(robotxr, robotyr);

%% path - resampling
revised_path = global_path';

r_cost = r_max + ones(1,culled_k_ls)*(r_robot + r_obs);
for i = 1:culled_k_ls
    for j = 1:360
        areax(j) = culled_C_ls(i,1) + r_cost(i)*cosd(j);
        areay(j) = culled_C_ls(i,2) + r_cost(i)*sind(j);
    end
%     hold on; plot(areax, areay);
end

for i = 1:culled_k_ls
    revise_idx = [];
    revise_target_vec = [];
    for j = 1:path_len-1
        if norm(revised_path(j+1,:) - culled_C_ls(i,:)) < r_cost(i)
            if isempty(revise_target_vec)
                revise_idx(1) = j+1;
                revise_target_vec(1,:) = revised_path(j+1,:) - culled_C_ls(i,:);
            else
                revise_idx(2) = j+1;
                revise_target_vec(2,:) = revised_path(j+1,:) - culled_C_ls(i,:);
            end
        end
    end
    if ~isempty(revise_target_vec)
        angle = acos(dot(revise_target_vec(1,:), revise_target_vec(2,:))/(norm(revise_target_vec(1,:))*norm(revise_target_vec(2,:))));
        angle_increment = angle/(revise_idx(2) - revise_idx(1) + 1);
    
        angle_init = atan2(revise_target_vec(1,2), revise_target_vec(1,1));
        db = goal_pose(2) - robot_pose(2);
        ca = goal_pose(1) - robot_pose(1);
        center_pos = db*culled_C_ls(i,1) - ca*culled_C_ls(i,2) + ca*robot_pose(2) - db*robot_pose(1);
        for k = revise_idx(1):revise_idx(2)
            if center_pos < 0
                th = angle_init + (k-revise_idx(1))*angle_increment;
                revised_path(k,:) = culled_C_ls(i,:) + [r_cost(i)*cos(th), r_cost(i)*sin(th)];
            elseif center_pos >= 0
                th = angle_init - (k-revise_idx(1))*angle_increment;
                revised_path(k,:) = culled_C_ls(i,:) + [r_cost(i)*cos(th), r_cost(i)*sin(th)];
            end
        end
    end
end

hold on; plot(revised_path(:,1), revised_path(:,2), 'o-');
xlim([0 17]); ylim([-1 3]);
xlabel("x"); ylabel("y"); title("path planning")
% set(fig1, 'OuterPosition', [250, 250, 700, 700])