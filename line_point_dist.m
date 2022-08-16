function [dist, ang] = line_point_dist(start_point, end_point, point)
db = end_point(2) - start_point(2);
ca = end_point(1) - start_point(1);
dist = (db*point(1) - ca*point(2) + ca*start_point(2) - db*start_point(1))/sqrt(db^2+ca^2);
ang = atan2(-ca,db)
if dist > 0 && ang < 0
    ang = ang+pi;
elseif dist < 0 && ang > 0
    ang = ang-pi;
end
dist = abs(dist);
end