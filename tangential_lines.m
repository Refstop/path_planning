function [tangential_point, argmin] = tangential_lines(close_C, pose, goal_pose, r_cost)
vec = close_C - pose;
a = norm(vec);
b = sqrt(a^2 - r_cost^2);
th = acos(b/a);
alpha = atan2(vec(2), vec(1));
phi = [alpha + th, alpha - th];
tangential_vector = [b*cos(phi(1)), b*sin(phi(1)); b*cos(phi(2)), b*sin(phi(2))];
tangential_point = [pose(1:2) + tangential_vector(1,:); pose(1:2) + tangential_vector(2,:)];
cur_path_vec = goal_pose(1:2) - pose;
phi = [acos(dot(cur_path_vec, tangential_vector(1,:))/(norm(cur_path_vec)*norm(tangential_vector(1,:)))), ...
acos(dot(cur_path_vec, tangential_vector(2,:))/(norm(cur_path_vec)*norm(tangential_vector(2,:))))];
[argval, argmin] = min(phi);
end