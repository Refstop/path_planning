function smooth_revised_path = smooth_path(revised_path)
smooth_revised_path = [];
grad_thres = 0.45; % param
spline_resol = 30; % param
spline_last = false;
for i = 2:length(revised_path)-1
    if spline_last
        spline_last = false;
        continue;
    end
    front_grad = atan2(revised_path(i,2) - revised_path(i-1,2), revised_path(i,1) - revised_path(i-1,1));
    back_grad = atan2(revised_path(i+1,2) - revised_path(i,2), revised_path(i+1,1) - revised_path(i,1));
    if abs(back_grad - front_grad) > grad_thres
        for j = 1:spline_resol
            t = (j-1)/spline_resol;
            smooth_revised_path(end+1,:) = [t^2, t, 1]*[1,-2,1; -2,2,0; 1,0,0]*[revised_path(i-1,:); revised_path(i,:); revised_path(i+1,:)];
            spline_last = true;
        end
    else
        smooth_revised_path(end+1,:) = revised_path(i,:);
    end
end
smooth_revised_path(end+1,:) = revised_path(end,:);

end