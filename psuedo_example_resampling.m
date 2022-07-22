clc;clear all;close all;
%{
조건1. 장애물'만' 클러스터링
조건2. 클러스터링 성능은 좋아야함
조건3. 그리드맵과 비교하여 inlier는 제거하는 과정 필요?
조건4. 동적 장애물도 커버 가능한가?
%}
%% pseudo laserscan
y1 = [1:0.01:12];
x1 = ones(1,length(y1));
x1_scan = x1 + 0.1*randn(1,length(x1));
y1_scan = y1 + 0.1*randn(1,length(y1));

y2 = [1:0.01:12];
x2 = ones(1,length(y2))*11;
x2_scan = x2 + 0.1*randn(1,length(x2));
y2_scan = y2 + 0.1*randn(1,length(y2));

x = [x1, x2];
y = [y1, y2];
x_scan = [x1_scan, x2_scan];
y_scan = [y1_scan, y2_scan];

y1_obj = [-1:0.01:16];
x1_obj = ones(1,length(y1_obj));
y2_obj = [-1:0.01:16];
x2_obj = ones(1,length(y2_obj))*11;

fig1 = figure(1);
plot(x1_obj, y1_obj, 'LineWidth', 3, 'Color', 'black');
hold on; plot(x2_obj, y2_obj, 'LineWidth', 3, 'Color', 'black');
% hold on; plot(x_scan, y_scan, 'o', 'MarkerSize', 1);
grid on;

%% pseudo obstacle
r = 1;
x_obs_1 = 5.5;
y_obs_1 = 6;

min_val_1 = 190;
max_val_1 = 350;
for i = min_val_1:max_val_1
    x3(i-min_val_1+1) = r*cosd(i);
    y3(i-min_val_1+1) = r*sind(i);
end
x3_scan = x3 + ones(1,length(x3))*x_obs_1 + 0.05*randn(1,length(x3));
y3_scan = y3 + ones(1,length(y3))*y_obs_1 + 0.05*randn(1,length(y3));

x_obs_2 = 6; % 6.64082
y_obs_2 = 9;

min_val_2 = 220;
max_val_2 = 350;
for i = min_val_2:max_val_2
    x4(i-min_val_2+1) = r*cosd(i);
    y4(i-min_val_2+1) = r*sind(i);
end
x4_scan = x4 + ones(1,length(x4))*x_obs_2 + 0.05*randn(1,length(x4));
y4_scan = y4 + ones(1,length(y4))*y_obs_2 + 0.05*randn(1,length(y4));

for i = 1:360
    x_obj(i) = r*cosd(i);
    y_obj(i) = r*sind(i);
end
x3_obj = x_obj + ones(1,360)*x_obs_1;
y3_obj = y_obj + ones(1,360)*y_obs_1;
x4_obj = x_obj + ones(1,360)*x_obs_2;
y4_obj = y_obj + ones(1,360)*y_obs_2;

hold on; plot(x3_obj, y3_obj, 'LineWidth', 2, 'Color', 'black');
hold on; plot(x4_obj, y4_obj, 'LineWidth', 2, 'Color', 'black');
hold on; plot(x3_scan, y3_scan, 'o', 'MarkerSize', 1, 'Color', 'red');
hold on; plot(x4_scan, y4_scan, 'o', 'MarkerSize', 1, 'Color', 'red');

%% inflation radius
r_obs=0.2;
x_total=[];
y_total=[];
for i = 1:length(x_scan)
    for j = 1:360
        xr(j) = x_scan(i) + r_obs*cosd(j);
        yr(j) = y_scan(i) + r_obs*sind(j);
    end
    x_total = [x_total, xr];
    y_total = [y_total, yr];
end

% hold on; plot(x_total, y_total, 'o', 'MarkerSize', 1);

x_totalr=[];
y_totalr=[];
for i = 1:length(x3_scan)
    for j = 1:360
        xr(j) = x3_scan(i) + r_obs*cosd(j);
        yr(j) = y3_scan(i) + r_obs*sind(j);
    end
    x_totalr = [x_totalr, xr];
    y_totalr = [y_totalr, yr];
end

% hold on; plot(x_totalr, y_totalr, 'o', 'MarkerSize', 1);

%% current robot & goal
robot_pose = [6.2, 2, 1.7];
goal_pose = [6.8, 14, 1.4];

robot = triangle(robot_pose(1), robot_pose(2), robot_pose(3));
hold on;
robot_pgon = polyshape(robot(1,:), robot(2,:));
plot(robot_pgon);
hold on; plot(robot(1,1), robot(2,1), '*');

goal = triangle(goal_pose(1), goal_pose(2), goal_pose(3));
hold on;
goal_pgon = polyshape(goal(1,:), goal(2,:));
plot(goal_pgon);
hold on; plot(goal(1,1), goal(2,1), '*');

%% global path(point to point)
path_len = 50;
global_path_x = linspace(robot_pose(1), goal_pose(1), path_len);
global_path_y = linspace(robot_pose(2), goal_pose(2), path_len);
global_path = [global_path_x; global_path_y];
hold on; plot(global_path_x, global_path_y, 'Color', 'red');

%% collision region of robot
r_robot = 0.6;
for i = 1:360
    robotxr(i) = robot_pose(1) + r_robot*cosd(i);
    robotyr(i) = robot_pose(2) + r_robot*sind(i);
end
hold on; plot(robotxr, robotyr);

%% dbscan clustering
X = [x1_scan, x2_scan, x3_scan, x4_scan; y1_scan, y2_scan, y3_scan, y4_scan]';
micro_error = [0.05, 0.05, 0.01];
tf = [cos(micro_error(3)), -sin(micro_error(3)) micro_error(1);
    sin(micro_error(3)), cos(micro_error(3)), micro_error(2);
    0, 0, 1];

micro_error_X = tf*[X';ones(1, length(X))];
X = micro_error_X(1:2,:)';

%% mycostmap
y1_map = [0:0.01:15];
x1_map = ones(1,length(y1_map));
x1_map = x1_map + 0.05*randn(1,length(y1_map));

y2_map = [0:0.01:15];
x2_map = ones(1,length(y2_map))*11;
x2_map = x2_map + 0.05*randn(1,length(y2_map));

x_map = [x1_map, x2_map];
y_map = [y1_map, y2_map];

%% filtering samples like laserscan
sensing_range = 10.5;
X_in_range = [];
xy_map_in_range = [];
for i = 1:length(X)
    if norm(X(i,:) - robot_pose(1:2)) < sensing_range
        X_in_range(end+1,:) = X(i,:);
    end
end
xy_map = [x_map; y_map]';
for i = 1:length(xy_map)
    if norm(xy_map(i,:) - robot_pose(1:2)) < sensing_range
        xy_map_in_range(end+1,:) = xy_map(i,:);
    end
end
hold on; plot(X_in_range(:,1), X_in_range(:,2), '.');
hold on; plot(xy_map_in_range(:,1), xy_map_in_range(:,2), '.');

idx_X = dbscan(X_in_range,1,5);
k_val_X = unique(idx_X);
k_X = length(k_val_X);
for i = 1:k_X
    C_X(i,:) = [mean(X_in_range(idx_X==i,1)), mean(X_in_range(idx_X==i,2))];
end
% hold on; plot(C_X(:,1), C_X(:,2), 'o', 'MarkerSize', 3);
idx_map = dbscan(xy_map_in_range,1,5);
k_val_map = unique(idx_map);
k_map = length(k_val_map);
for i = 1:k_map
    C_map(i,:) = [mean(xy_map_in_range(idx_map==i,1)), mean(xy_map_in_range(idx_map==i,2))];
end
% hold on; plot(C_map(:,1), C_map(:,2), 'o', 'MarkerSize', 3);

%% culling obstacles from laserscan by cosine similarity
for i = 1:length(C_X)
    for j = 1:length(C_map)
        cos_sim(i,j) = (dot(C_X(i,:), C_map(j,:)))/(norm(C_X(i,:))*norm(C_map(j,:)));
    end
end
culling_idx = [];
culling_threshold = 0.8;
for i = 1:length(C_map)
    [argval, argmax] = max(cos_sim(:,i));
    if argval > culling_threshold
        culling_idx(end+1) = argmax;
        k_X = k_X - 1;
    end
end
culled_k_val_X = k_val_X;
culled_k_val_X(culling_idx) = [];
culled_C_X = C_X;
culled_C_X(culling_idx,:) = [];
hold on; plot(C_X(:,1), C_X(:,2), 'o', 'MarkerSize', 5);

culling_point_idx = [];
for i = 1:length(idx_X)
    for j = 1:length(culling_idx)
        if idx_X(i) == culling_idx(j)
             culling_point_idx(end+1) = i;
        end
    end
end

culled_X = X_in_range;
culled_X(culling_point_idx,:) = [];
culled_idx_X = idx_X;
culled_idx_X(culling_point_idx,:) = [];


%% path - resampling
revised_path = global_path';
for i = 1:k_X
    range = [];
    for j = 1:length(culled_idx_X)
        if culled_idx_X(j) == culled_k_val_X(i)
            range(end+1) = norm(culled_C_X(i,:) - culled_X(j,:));
        end
    end
    r_max(i) = max(range);
end

r_cost = r_max + ones(1,k_X)*(r_robot + r_obs);
for i = 1:k_X
    for j = 1:360
        areax(j) = culled_C_X(i,1) + r_cost(i)*cosd(j);
        areay(j) = culled_C_X(i,2) + r_cost(i)*sind(j);
    end
    hold on; plot(areax, areay);
end

for i = 1:k_X
    revise_idx = [];
    revise_target_vec = [];
    for j = 1:path_len-1
        if norm(revised_path(j+1,:) - culled_C_X(i,:)) < r_cost(i)
            if isempty(revise_target_vec)
                revise_idx(1) = j+1;
                revise_target_vec(1,:) = revised_path(j+1,:) - culled_C_X(i,:);
            else
                revise_idx(2) = j+1;
                revise_target_vec(2,:) = revised_path(j+1,:) - culled_C_X(i,:);
            end
        end
    end
    if ~isempty(revise_target_vec)
        angle = acos(dot(revise_target_vec(1,:), revise_target_vec(2,:))/(norm(revise_target_vec(1,:))*norm(revise_target_vec(2,:))));
        angle_increment = angle/(revise_idx(2) - revise_idx(1) + 1);
    
        angle_init = atan2(revise_target_vec(1,2), revise_target_vec(1,1));
        db = goal_pose(2) - robot_pose(2);
        ca = goal_pose(1) - robot_pose(1);
        center_pos = db*culled_C_X(i,1) - ca*culled_C_X(i,2) + ca*robot_pose(2) - db*robot_pose(1);
        for k = revise_idx(1):revise_idx(2)
            if center_pos < 0
                th = angle_init + (k-revise_idx(1))*angle_increment;
                revised_path(k,:) = culled_C_X(i,:) + [r_cost(i)*cos(th), r_cost(i)*sin(th)];
            elseif center_pos >= 0
                th = angle_init - (k-revise_idx(1))*angle_increment;
                revised_path(k,:) = culled_C_X(i,:) + [r_cost(i)*cos(th), r_cost(i)*sin(th)];
            end
        end
    end
end

hold on; plot(revised_path(:,1), revised_path(:,2), 'o');
xlim([0 15]); ylim([0 15]);
xlabel("x"); ylabel("y"); title("path planning")
set(fig1, 'OuterPosition', [250, 250, 700, 700])

%% before culling & after culling
drawArrow = @(x,y,color) quiver( x(1),y(1),x(2)-x(1),y(2)-y(1),0 , 'LineWidth', 2, 'Color', color);

fig2 = figure(2);
subplot(131)

plot(x1_obj, y1_obj, 'LineWidth', 3, 'Color', 'black');
hold on; plot(x2_obj, y2_obj, 'LineWidth', 3, 'Color', 'black');
hold on; plot(x3_obj, y3_obj, 'LineWidth', 3, 'Color', 'black');
hold on; plot(x4_obj, y4_obj, 'LineWidth', 3, 'Color', 'black');
hold on; plot(xy_map_in_range(:,1), xy_map_in_range(:,2), '.');
for i = 1:length(k_val_map)
    hold on; drawArrow([0, C_map(i,1)], [0, C_map(i,2)], 'red')
end
xlim([0 15]); ylim([0 15]);
xlabel("x"); ylabel("y"); title("gridmap's point in sensing range")
grid on;

subplot(132)

plot(x1_obj, y1_obj, 'LineWidth', 3, 'Color', 'black');
hold on; plot(x2_obj, y2_obj, 'LineWidth', 3, 'Color', 'black');
hold on; plot(x3_obj, y3_obj, 'LineWidth', 3, 'Color', 'black');
hold on; plot(x4_obj, y4_obj, 'LineWidth', 3, 'Color', 'black');
hold on; plot(X_in_range(:,1), X_in_range(:,2), '.');
for i = 1:length(k_val_X)
    hold on; drawArrow([0, C_X(i,1)], [0, C_X(i,2)], 'blue')
end
for i = 1:length(k_val_map)
    hold on; drawArrow([0, C_map(i,1)], [0, C_map(i,2)], 'red')
end
xlim([0 15]); ylim([0 15]);
xlabel("x"); ylabel("y"); title("before culling(only laserscan)")
grid on;

subplot(133)

plot(x1_obj, y1_obj, 'LineWidth', 3, 'Color', 'black');
hold on; plot(x2_obj, y2_obj, 'LineWidth', 3, 'Color', 'black');
hold on; plot(x3_obj, y3_obj, 'LineWidth', 3, 'Color', 'black');
hold on; plot(x4_obj, y4_obj, 'LineWidth', 3, 'Color', 'black');
hold on; plot(culled_X(:,1), culled_X(:,2), '.');
for i = 1:length(culled_k_val_X)
    hold on; drawArrow([0, culled_C_X(i,1)], [0, culled_C_X(i,2)], 'blue')
end
xlim([0 15]); ylim([0 15]);
xlabel("x"); ylabel("y"); title("after culling")
grid on;

set(fig2, 'OuterPosition', [250, 250, 1500, 500])