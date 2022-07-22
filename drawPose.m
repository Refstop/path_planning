function target_origin = drawCoord(map, reference_coord, target_coord, draw)
    T_or = tf2d(reference_coord);
    T_rt = tf2d(target_coord);
    origin = [0;0;1];
    if draw
        for i = 1:5
            origin(:,end+1) = [0;0.05*i;1];
            origin(:,end+1) = [0.05*i;0;1];
        end
        target_origin_coord = T_or*T_rt*origin;
        for i = 1:length(target_origin_coord)
            setOccupancy(map, target_origin_coord(1:2,i)', 1);
        end
    else
        target_origin_coord = T_or*T_rt*origin;
    end
    target_origin = target_origin_coord(1:2,1);
end