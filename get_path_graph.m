function [path_graph, path_node] = get_path_graph(parent_node, path_node, i, new_obs, G, goal_pose, culled_k_ls, close_C_ls, r_cost, fig)
if i == culled_k_ls+1
    path_graph = G;
    return;
end

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
    
    if new_obs
        cur_robot_pose = cell2mat(G.Nodes.path(parent_node));
        new_obs = false;
    else
        if free_from_obs(culled_k_ls, i, close_C_ls, point_start_avoid, r_cost(i))
            G = addnode(G,1);
            G.Nodes.path(path_node) = {point_start_avoid};
            G = addedge(G,parent_node, path_node);
            parent_node = path_node;
            path_node = path_node+1;
            cur_robot_pose = point_start_avoid;
        end
    end
    

    % 2. 1번의 시작 지점을 스타트로 inflation radius에 접하는 경로 생성 - 여기서부터 분기점 시작
    sample_num = 100;
    % 현재 로봇의 위치에서 다음 장애물에 대한 접점 2개와 각 접점에 대한 각도를 추출
    [tangential_point, phi] = tangential_lines(close_C_ls(i,:), cur_robot_pose, point_end_avoid, r_cost(i));

    branch = [];
    % 두쪽 다에 대해서 검사
    for j = 1:2
        line_sampling = [linspace(cur_robot_pose(1), tangential_point(j,1), sample_num)', linspace(cur_robot_pose(2), tangential_point(j,2), sample_num)'];
%         fprintf("cur: %f, %f\ntan: %f, %f\n\n", cur_robot_pose(1), cur_robot_pose(2), tangential_point(j,1), tangential_point(j,2));
        hold on; plot(tangential_point(j,1), tangential_point(j,2), 'o');
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
            G.Nodes.path(path_node) = {tangential_point(j,:)};
            G = addedge(G, parent_node, path_node);
            branch(end+1) = path_node;
            path_node = path_node+1;
        end
    end

    % 둘 다 갈수 없는 경우 branch가 비어버린다.
    for j = 1:length(branch)
        cur_robot_pose = cell2mat(G.Nodes.path(branch(j)));
        [tangential_point, phi] = tangential_lines(close_C_ls(i,:), point_end_avoid, cur_robot_pose, r_cost(i));

        % 경로별 매치 필요
        [argval, argmin] = min([norm(tangential_point(1,:) - cur_robot_pose), norm(tangential_point(2,:) - cur_robot_pose)]);
        hold on; plot(tangential_point(argmin,1), tangential_point(argmin,2), 'o');

        vec1 = cur_robot_pose - close_C_ls(i,:);
        vec2 = tangential_point(argmin,:) - close_C_ls(i,:);

        path_angle = acos(dot(vec1, vec2)/(norm(vec1)*norm(vec2)));
        num = 8; % param
        angle_inc = path_angle/num;
        angle_init = atan2(vec1(2), vec1(1));

        parent_node = branch(j);
        for l = 1:num
            if j==1 % hard code
                th_rspl = angle_init - l*angle_inc;
            else
                th_rspl = angle_init + l*angle_inc;
            end

            point_rspl = close_C_ls(i,:) + [r_cost(i)*cos(th_rspl), r_cost(i)*sin(th_rspl)];
            if ~free_from_obs(culled_k_ls, i, close_C_ls, point_rspl, r_avoid)
                new_obs = true;
                [G, path_node] = get_path_graph(parent_node, path_node, i+1, new_obs, G, goal_pose, culled_k_ls, close_C_ls, r_cost, fig);
                break;
            end
            hold on; plot(point_rspl(1), point_rspl(2), 'o');
            G = addnode(G,1);
            G.Nodes.path(path_node) = {point_rspl};
            G = addedge(G, parent_node, path_node);
            parent_node = path_node;
            path_node = path_node+1;
        end
        fprintf("parent: %d, path: %d\n", parent_node, path_node);
        if ~new_obs && free_from_obs(culled_k_ls, i, close_C_ls, point_end_avoid, r_avoid)
            G = addnode(G,1);
            G.Nodes.path(path_node) = {point_end_avoid};
            G = addedge(G, parent_node, path_node);
            parent_node = path_node;
            path_node = path_node+1;
        else
            [G, path_node] = get_path_graph(parent_node, path_node, i+1, new_obs, G, goal_pose, culled_k_ls, close_C_ls, r_cost, fig);
        end
        
    end
    path_graph = G; return;
end
end