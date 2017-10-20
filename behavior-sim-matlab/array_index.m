function [r_index, v_index, F_index] = array_index(i,p)
    % Indexes for the current vehicle
    r_index = (1:p.n)'+p.n*(i-1);
    v_index = (1:p.m)'+p.m*(i-1);
    F_index = (1:p.m)'+p.m*(i-1);
end

