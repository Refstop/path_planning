function check = free_from_obs(k, cur_idx, close_C_ls, point, r_cost) % change name
check = true;
for i = 1:k
    if (norm(close_C_ls(i,:) - point) <= r_cost) && (i ~= cur_idx)
        check = false;
        return;
    end
end
end