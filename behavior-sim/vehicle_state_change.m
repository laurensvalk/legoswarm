function r_dot = vehicle_state_change(r, v, p)
% Calculate the state change, given the current state and the input
    
    % Extract the angles
    theta = r(p.n:p.n:end);

    % Empty output
    r_dot = zeros(p.n*p.N,1);
    
    % For each vehicle compute the cartesian speed
    for i = 1:p.N
        
        % Indexes for the current vehicle
        [r_index, v_index, ~] = array_index(i,p);
        
        % Velocity jacobian for the current vehicle
        J = [cos(theta(i)), 0;
             sin(theta(i)), 0;
             0            , 1];        
        
        % Cartesian velocity for the current vehicle
        r_dot(r_index) = J*v(v_index);    
    end
end

