clc;clear all;close all;
% 현재 문제점: waypoint 근처에 장애물이 있을 때?
% 장애물들 사이의 거리가 너무 가깝다면 빙 돌아가는 코스 만들기
% 운행 도중 목적지가 바뀌면(경로가 바뀌면) 바로 수정하도록 하기

%% costmap
map_origin = [-48, -8, 0];
image = imread('hitech6.pgm');
imageNorm = double(image)/255;
imageOccupancy = 1 - imageNorm;
map = occupancyMap(imageOccupancy,20);

%% robot, lidar data load
laserscan = load("data.txt");
robot_pose = load("pose.txt");
angle_min = -2.05370092392;
angle_max = 2.05370092392;
angle_increment = 0.00581718236208;
angle_n = (angle_max - angle_min)/angle_increment;

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
set(fig3, 'OuterPosition', [250, 250, 1000, 450])
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
r_robot = 0.3;
for i = 1:360
    robotxr(i) = robot_pose(1) + r_robot*cosd(i);
    robotyr(i) = robot_pose(2) + r_robot*sind(i);
end
hold on; plot(robotxr, robotyr);

%% path - resampling
r_cost = r_max + ones(1,culled_k_ls)*(r_robot + r_obs);
for i = 1:culled_k_ls
    for j = 1:360
        areax(j) = culled_C_ls(i,1) + r_cost(i)*cosd(j);
        areay(j) = culled_C_ls(i,2) + r_cost(i)*sind(j);
    end
    hold on; plot(areax, areay);
end

%% planning(each iters(seconds))
close_C_ls = culled_C_ls; % 가까운 순으로 정렬 - 생략
cur_robot_pose = global_path(:,1)';
revised_path(1,:) = global_path(:,1)';
path_reverse = false;
sampling_resolution = 0.03;

for i = 1:culled_k_ls
    if i == 1
        % robot-obs
        vec = close_C_ls(i,:) - cur_robot_pose;
        a = norm(vec);
        b = sqrt(a^2 - r_cost(i)^2);
        th = acos(b/a);
        alpha = atan2(vec(2), vec(1));
        phi = [alpha + th, alpha - th];
        tangential_vector = [b*cos(phi(1)), b*sin(phi(1)); b*cos(phi(2)), b*sin(phi(2))];
        tangential_point = [cur_robot_pose(1:2) + tangential_vector(1,:); cur_robot_pose(1:2) + tangential_vector(2,:)];
        cur_path_vec = goal_pose(1:2) - cur_robot_pose;
        phi = [acos(dot(cur_path_vec, tangential_vector(1,:))/(norm(cur_path_vec)*norm(tangential_vector(1,:)))), ...
        acos(dot(cur_path_vec, tangential_vector(2,:))/(norm(cur_path_vec)*norm(tangential_vector(2,:))))];

        [argval, argmin] = min(phi);
        revised_path(end+1,:) = tangential_point(argmin,:);

        % obs-goal
        vec = close_C_ls(i,:) - goal_pose(1:2);
        a = norm(vec);
        b = sqrt(a^2 - r_cost(i)^2);
        th = acos(b/a);
        alpha = atan2(vec(2), vec(1));
        phi = [alpha + th, alpha - th];
        tangential_vector = [b*cos(phi(1)), b*sin(phi(1)); b*cos(phi(2)), b*sin(phi(2))];
        tangential_point = [goal_pose(1:2) + tangential_vector(1,:); goal_pose(1:2) + tangential_vector(2,:)];
        cur_path_vec = cur_robot_pose - goal_pose(1:2);
        phi = [acos(dot(cur_path_vec, tangential_vector(1,:))/(norm(cur_path_vec)*norm(tangential_vector(1,:)))), ...
        acos(dot(cur_path_vec, tangential_vector(2,:))/(norm(cur_path_vec)*norm(tangential_vector(2,:))))];

        [argval, argmin] = min(phi);
        
        vec1 = revised_path(end,:) - culled_C_ls(i,:);
        vec2 = tangential_point(argmin,:) - culled_C_ls(i,:);
        
        path_angle = acos(dot(vec1, vec2)/(norm(vec1)*norm(vec2)));
        num = 5;
        angle_inc = path_angle/num;
        angle_init = atan2(vec1(2), vec1(1));
        for j = 1:num
            theta = angle_init - (j-1)*angle_inc;
            revised_path(end+1,:) = culled_C_ls(i,:) + [r_cost(i)*cos(theta), r_cost(i)*sin(theta)];
        end
        
    else
        db = goal_pose(2) - cur_robot_pose(2);
        ca = goal_pose(1) - cur_robot_pose(1);
        dist = abs(db*culled_C_ls(i,1) - ca*culled_C_ls(i,2) + ca*robot_pose(2) - db*robot_pose(1))/sqrt(db^2+ca^2);
        if dist < r_cost(i)
            vec = close_C_ls(i,:) - cur_robot_pose;
            a = norm(vec);
            b = sqrt(a^2 - r_cost(i)^2);
            th = acos(b/a);
            alpha = atan2(vec(2), vec(1));
            phi = [alpha + th, alpha - th];
            tangential_vector = [b*cos(phi(1)), b*sin(phi(1)); b*cos(phi(2)), b*sin(phi(2))];
            tangential_point = [cur_robot_pose(1:2) + tangential_vector(1,:); cur_robot_pose(1:2) + tangential_vector(2,:)];
            cur_path_vec = goal_pose(1:2) - cur_robot_pose;
            phi = [acos(dot(cur_path_vec, tangential_vector(1,:))/(norm(cur_path_vec)*norm(tangential_vector(1,:)))), ...
            acos(dot(cur_path_vec, tangential_vector(2,:))/(norm(cur_path_vec)*norm(tangential_vector(2,:))))];

            [argval, argmin] = min(phi);
            revised_path(end+1,:) = tangential_point(argmin,:);
            for j = 1:culled_k_ls
                if norm(revised_path(end,:) - culled_C_ls(j,:)) < r_cost(j)
                    revised_path(end,:) = tangential_point(rem(argmin,2)+1,:); %C++에선 나머지연산으로 스위칭
                    path_reverse = true;
                end
            end

            vec = close_C_ls(i,:) - goal_pose(1:2);
            a = norm(vec);
            b = sqrt(a^2 - r_cost(i)^2);
            th = acos(b/a);
            alpha = atan2(vec(2), vec(1));
            phi = [alpha + th, alpha - th];
            tangential_vector = [b*cos(phi(1)), b*sin(phi(1)); b*cos(phi(2)), b*sin(phi(2))];
            tangential_point = [goal_pose(1:2) + tangential_vector(1,:); goal_pose(1:2) + tangential_vector(2,:)];
            cur_path_vec = cur_robot_pose - goal_pose(1:2);
            phi = [acos(dot(cur_path_vec, tangential_vector(1,:))/(norm(cur_path_vec)*norm(tangential_vector(1,:)))), ...
            acos(dot(cur_path_vec, tangential_vector(2,:))/(norm(cur_path_vec)*norm(tangential_vector(2,:))))];

            [argval, argmin] = min(phi);
            num = 10;
            vec1 = revised_path(end,:) - culled_C_ls(i,:);
            if path_reverse
                vec2 = tangential_point(rem(argmin,2)+1,:) - culled_C_ls(i,:);
                path_reverse = false;
            else
                vec2 = tangential_point(argmin,:) - culled_C_ls(i,:);
            end
            path_angle = acos(dot(vec1, vec2)/(norm(vec1)*norm(vec2)));
            angle_inc = path_angle/num;
            angle_init = atan2(vec1(2), vec1(1));
            for j = 1:num
                theta = angle_init - (j-1)*angle_inc;
                revised_path(end+1,:) = culled_C_ls(i,:) + [r_cost(i)*cos(theta), r_cost(i)*sin(theta)];
            end
        end
    end
    cur_robot_pose = revised_path(end,:);
end
revised_path(end+1,:) = goal_pose(1:2);
hold on; plot(revised_path(:,1), revised_path(:,2), 'o-');
xlim([0 10]); ylim([-1 3]);
xlabel("x"); ylabel("y"); title("path planning")