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
x1 = x1 + 0.1*randn(1,length(y1));

y2 = [1:0.01:9];
x2 = ones(1,length(y2))*11;
x2 = x2 + 0.1*randn(1,length(y2));

x = [x1, x2];
y = [y1, y2];

subplot(121);
plot(x, y, 'o', 'MarkerSize', 1);
grid on;

%% pseudo obstacle
r=1;
x_obs_1 = 5.5;
y_obs_1 = 6;
for i = 120:360
    x3(i) = r*cosd(i);
    y3(i) = r*sind(i);
end
x3 = x3 + ones(1,length(x3))*x_obs_1 + 0.05*randn(1,length(x3));
y3 = y3 + ones(1,length(y3))*y_obs_1 + 0.05*randn(1,length(y3));

x_obs_2 = 7;
y_obs_2 = 11;

for i = 120:360
    x4(i) = r*cosd(i);
    y4(i) = r*sind(i);
end
x4 = x4 + ones(1,length(x4))*x_obs_2 + 0.05*randn(1,length(x4));
y4 = y4 + ones(1,length(y4))*y_obs_2 + 0.05*randn(1,length(y4));

hold on; plot(x3, y3, 'o', 'MarkerSize', 1);
hold on; plot(x4, y4, 'o', 'MarkerSize', 1);

%% inflation radius
r_obs=0.2;
x_total=[];
y_total=[];
for i = 1:length(x)
    for j = 1:360
        xr(j) = x(i) + r_obs*cosd(j);
        yr(j) = y(i) + r_obs*sind(j);
    end
    x_total = [x_total, xr];
    y_total = [y_total, yr];
end

% hold on; plot(x_total, y_total, 'o', 'MarkerSize', 1);

x_totalr=[];
y_totalr=[];
for i = 1:length(x3)
    for j = 1:360
        xr(j) = x3(i) + r_obs*cosd(j);
        yr(j) = y3(i) + r_obs*sind(j);
    end
    x_totalr = [x_totalr, xr];
    y_totalr = [y_totalr, yr];
end

% hold on; plot(x_totalr, y_totalr, 'o', 'MarkerSize', 1);

%% current robot & goal
robot_pose = [6.2, 2, 1.7];
robot = triangle(robot_pose(1), robot_pose(2), robot_pose(3));
hold on;
robot_pgon = polyshape(robot(1,:), robot(2,:));
plot(robot_pgon);
hold on; plot(robot(1,1), robot(2,1), '*');

goal_pose = [6.8, 14, 1.4];
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
% hold on; plot(global_path_x, global_path_y);

%% collision region of robot
r_robot=0.6;
for i = 1:360
    robotxr(i) = robot_pose(1) + r_robot*cosd(i);
    robotyr(i) = robot_pose(2) + r_robot*sind(i);
end
hold on;
plot(robotxr, robotyr);

%% kmeans clustering
X = [x3, x4; y3, y4]';
k=2;
[idx,C] = kmeans(X,k);
hold on;
plot(C(:,1), C(:,2), 'o', 'MarkerSize', 3, 'Color', 'red');

%% path - threshold 버전(?)
revised_path = global_path';
for i = 1:k
    range = [];
    for j = 1:length(idx)
        if idx(j) == i
            range(j) = norm(C(i,:) - X(j,:));
        end
    end
    r_max(i) = max(range);
end
r_cost = r_max + ones(1,k)*(r_robot + r_obs);
for i = 1:k
    for j = 1:360
        areax(j) = C(i,1) + r_cost(i)*cosd(j);
        areay(j) = C(i,2) + r_cost(i)*sind(j);
    end
    hold on; plot(areax, areay);
end

for i = 1:path_len-1
    th = atan2(revised_path(i+1,2) - revised_path(i,2), revised_path(i+1,1) - revised_path(i,1));
    robot_pose = [revised_path(i,1), revised_path(i,2), th];
    for j = 1:k
        if norm(revised_path(i+1,:) - C(j,:)) < r_cost(j)
            vec = revised_path(i+1,:) - C(j,:);
            projection_vec = vec/norm(vec)*r_cost(j);
            revised_path(i+1,:) = C(j,:) + projection_vec;
        end
    end
    if rem(i,4) == 0
        th = atan2(revised_path(i+1,2) - revised_path(i,2), revised_path(i+1,1) - revised_path(i,1));
        robot_path = triangle(revised_path(i,1), revised_path(i,2), th);
        hold on;
        robot_path_pgon = polyshape(robot_path(1,:), robot_path(2,:));
        plot(robot_path_pgon);
        hold on; plot(robot_path(1,1), robot_path(2,1), '*');
    end
end
hold on; plot(revised_path(:,1), revised_path(:,2), 'o-');
xlim([0 15]); ylim([0 15]);
xlabel("x"); ylabel("y"); title("path planning")

%% mycostmap
y1_map = [1:0.01:12];
x1_map = ones(1,length(y1_map));
x1_map = x1_map + 0.05*randn(1,length(y1_map));

y2_map = [1:0.01:9];
x2_map = ones(1,length(y2_map))*11;
x2_map = x2_map + 0.05*randn(1,length(y2_map));

x_map = [x1_map, x2_map];
y_map = [y1_map, y2_map];

map = occupancyMap(15,15,15);
xy_map = [x_map; y_map]';
pvalue = ones(length(x_map), 1)*0.2;
updateOccupancy(map, xy_map, pvalue);

subplot(122);
show(map);

%% scanmatching
currentScanCart = [x1, x2, x3, x4; y1, y2, y3, y4]';
th = pi/10; trans_x = 0.5; trans_y = 0.3;
T = [cos(th), -sin(th), trans_x;
    sin(th), cos(th), trans_y;
    0, 0, 1];
smalltf_curScan = T*[currentScanCart'; ones(1, length(currentScanCart))];
currentScanCart2 = smalltf_curScan(1:2,:)';
referenceScanCart = [x_map; y_map]';

currentScan = lidarScan(currentScanCart2);
referenceScan = lidarScan(referenceScanCart);
transform = matchScans(currentScan,referenceScan);
transScan = transformScan(currentScan,transform);
figure(1)
plot(currentScanCart2(:,1),currentScanCart2(:,2),'g.');
hold on
plot(referenceScanCart(:,1),referenceScanCart(:,2),'k.');
hold on
transScanCart = transScan.Cartesian;
plot(transScanCart(:,1),transScanCart(:,2),'r.');
legend('Reference laser scan','Transformed current laser scan','Location','NorthWest');
xlim([0 15]); ylim([0 15]);
grid on;

%% dbscan
figure(2)
data = [x1, x2, x3, x4; y1, y2, y3, y4]';
idx = dbscan(data,1,5); % The default distance metric is Euclidean distance
gscatter(data(:,1),data(:,2),idx);
title('DBSCAN Using Euclidean Distance Metric')