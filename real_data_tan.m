clc;clear all;close all;

%% costmap
map_origin = [-48, -8, 0];
image = imread('hitech6.pgm');
imageNorm = double(image)/255;
imageOccupancy = 1 - imageNorm;
map = occupancyMap(imageOccupancy,20);

%% robot, lidar data load
laserscan = load("data1.txt");
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

fig1 = figure(1);
plot(laserscan(:,1), laserscan(:,2), '.')
set(fig1, 'OuterPosition', [250, 250, 1000, 450])
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
    C_ls_ogm = T_om*[C_ls(i,1); C_ls(i,2); 1];
    x = []; y = [];
    for j = 1:angle_n
        angle = angle_min + (j-1)*angle_increment;
        x(j,:) = C_ls_ogm(1) + ranges*cos(angle);
        y(j,:) = C_ls_ogm(2) + ranges*sin(angle);
    end

    window_size = size(x);
    
    % occupied_cell = [];
    occupied_num = 0;
    for j = 1:window_size(1)
        for k = 1:window_size(2)
            cur_cell_val = getOccupancy(map, [x(j,k), y(j,k)]);
            if cur_cell_val > cell_val_threshold
                % occupied_cell(:,end+1) = [x(j,k); y(j,k); 1];
                occupied_num = occupied_num + 1;
            end
        end
    end
    % if length(occupied_cell) > 20
    if occupied_num > 20
        culled_laserscan(culled_idx_ls == k_val_ls(i),:) = [];
        culled_idx_ls(culled_idx_ls == k_val_ls(i)) = [];
        culling_index(end+1) = i;
        % points matched with map
    end
end
culled_k_val_ls(culling_index) = [];
culled_k_ls = length(culled_k_val_ls);
culled_C_ls(culling_index,:) = [];
r_max(culling_index) = [];
% hold on; plot(culled_laserscan(:,1), culled_laserscan(:,2), '.');
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
path_len = 10; % adaptive
global_path_x = linspace(robot_pose(1), goal_pose(1), path_len);
global_path_y = linspace(robot_pose(2), goal_pose(2), path_len);
global_path = [global_path_x; global_path_y];
hold on; plot(global_path_x, global_path_y, '-', 'Color', 'red');

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
sampling_resolution = 0.03;

% graph initialization
G = digraph;
path_node_idx = 1;
G = addnode(G,1);
G.Nodes.path(path_node_idx) = {global_path(:,1)'};
path_node_idx = path_node_idx + 1;

xlim([-0.1 7.1]); ylim([-0.6 2.5]);
path_graph = get_path_graph(path_node_idx, 1, false, G, goal_pose, culled_k_ls, close_C_ls, r_cost, fig1);
figure; plot(path_graph); return;

for i = 1:culled_k_ls
    r_avoid = r_cost(i) + 0.2; % param
    for j = 1:360
        hold on; plot(close_C_ls(i,1)+r_avoid*cosd(j), close_C_ls(i,2)+r_avoid*sind(j), '.');
    end
    % global 직선 경로와 인식된 장애물 중심 간의 거리와 원 중심으로부터 직선 경로에 내리는 수직 벡터의 각도
    [dist, ang] = line_point_dist(cell2mat(G.Nodes.path(1)), goal_pose(1:2), close_C_ls(i,:));
    
    if dist < r_cost(i)
        % 1. 직선 경로 상의 장애물 회피 시작 지점 결정
        if ang < 0
            th_avoid = [ang - acos(dist/r_avoid), ang + acos(dist/r_avoid)];
        else
            th_avoid = [ang + acos(dist/r_avoid), ang - acos(dist/r_avoid)];
        end
        
        vec_avoid = [r_avoid*cos(th_avoid(1)), r_avoid*sin(th_avoid(1)); ...
                    r_avoid*cos(th_avoid(2)), r_avoid*sin(th_avoid(2))];
        point_start_avoid = close_C_ls(i,:) + vec_avoid(1,:);
        point_end_avoid = close_C_ls(i,:) + vec_avoid(2,:); % 도착지점도 장애물 반경 안에 드가는지 체크
        hold on; plot(point_start_avoid(1), point_start_avoid(2),'o','MarkerSize',12);
        hold on; plot(point_end_avoid(1), point_end_avoid(2),'o','MarkerSize',12);
        
        if free_from_obs(culled_k_ls, i, close_C_ls, point_start_avoid, r_cost(i))
            % revised_path(end+1,:) = point_start_avoid;
            G = addnode(G,1);
            G.Nodes.path(path_node_idx) = {point_start_avoid};
            G = addedge(G,path_node_idx-1,path_node_idx);

            path_node_idx = path_node_idx + 1;
        end
        cur_robot_pose = point_start_avoid;
        
        % 2. 1번의 시작 지점을 스타트로 inflation radius에 접하는 경로 생성 - 여기서부터 분기점 시작 
        sample_num = 100;
        % 현재 로봇의 위치에서 다음 장애물에 대한 접점 2개와 각 접점에 대한 각도를 추출
        [tangential_point, phi] = tangential_lines(close_C_ls(i,:), cur_robot_pose, point_end_avoid, r_cost(i));
        
        parent_node_idx = path_node_idx;
        branch = [];
        % 두쪽 다에 대해서 검사
        for j = 1:2
            line_sampling = [linspace(cur_robot_pose(1), tangential_point(j,1), sample_num)', linspace(cur_robot_pose(2), tangential_point(j,2), sample_num)'];
            
            f_check = true;
            for k = 1:sample_num
                check = free_from_obs(culled_k_ls, i, close_C_ls, line_sampling(k,:), r_cost(i));
                if ~check
                    f_check = check;
                    break;
                end
            end
            if f_check
                G = addnode(G,1);
                G.Nodes.path(path_node_idx) = {tangential_point(j,:)};
                G = addedge(G, parent_node_idx-1, path_node_idx);
                branch(end+1) = path_node_idx;
                path_node_idx = path_node_idx+1;
                hold on; plot(tangential_point(j,1), tangential_point(j,2), 'o');
            end
        end
        
        for j = 1:2
            cur_robot_pose = cell2mat(G.Nodes.path(branch(j)));
            [tangential_point, phi] = tangential_lines(close_C_ls(i,:), point_end_avoid, cur_robot_pose, r_cost(i));
            % 경로별 매치 필요
            vec1 = cur_robot_pose - close_C_ls(i,:);
            vec2 = tangential_point(j,:) - close_C_ls(i,:);
                
            path_angle = acos(dot(vec1, vec2)/(norm(vec1)*norm(vec2)));
            num = 8; % param
            angle_inc = path_angle/num;
            angle_init = atan2(vec1(2), vec1(1));
            
            parent_node_idx = branch(j);
            for l = 1:num
                if j==1 % 하드코딩, 나중에 고칠것 
                    th_rspl = angle_init - l*angle_inc;
                else
                    th_rspl = angle_init + l*angle_inc;
                end

                point_rspl = close_C_ls(i,:) + [r_cost(i)*cos(th_rspl), r_cost(i)*sin(th_rspl)];
                hold on; plot(point_rspl(1), point_rspl(2), 'o');
                if ~free_from_obs(culled_k_ls, i, close_C_ls, point_rspl, r_avoid) % new obs
                    break;
                end
                G = addnode(G,1);
                G.Nodes.path(path_node_idx) = {point_rspl};
                G = addedge(G, parent_node_idx, path_node_idx);
                parent_node_idx = path_node_idx;
                path_node_idx = path_node_idx+1;
                % revised_path(end+1,:) = point_rspl;
            end
        end
        figure; plot(G); return; 
        cur_robot_pose
    end
end
revised_path(end+1,:) = goal_pose(1:2);

%% spline 추가(bezier spline)
smooth_revised_path = smooth_path(revised_path);

hold on; plot(smooth_revised_path(:,1), smooth_revised_path(:,2), "LineWidth", 2);
hold on; plot(revised_path(:,1), revised_path(:,2), 'o-');
% xlim([0 10]); ylim([-1 3]);
xlim([-0.1 7.1]); ylim([-0.6 2.5]);
xlabel("x"); ylabel("y"); title("path planning")