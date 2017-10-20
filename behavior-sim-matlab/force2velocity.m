function v = force2velocity(F,r,p)
% Convert a force acting on the front connector to the corresponding output
% velocity, given the current state

    % Extract the angles
    theta = r(p.n:p.n:end);
    
    % Empty output
    v = zeros(p.m*p.N,1);    
    
    % For each vehicle 
    for i = 1:p.N
        [~, v_index, F_index] = array_index(i,p);
        
        % Rotation matrix defined in this way: world = R * robot_i
        R_w_i = [cos(theta(i)), -sin(theta(i));
                 sin(theta(i)), cos(theta(i))];
        
        % Force in robot coordinates
        F_now = R_w_i'*F(F_index);
        
        % Split the force in components
        F_forward = F_now(1);
        Torque    = F_now(2)*p.a;
        
        % Velocities
        s = F_forward * p.f2v;
        theta_dot = Torque * p.t2h;
        
        % Store the velocities for this robot
        v(v_index) = [s; theta_dot];
    end    
    
    

end


