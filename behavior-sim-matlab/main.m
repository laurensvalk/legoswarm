%% Setup

% Clear memory
clear all; clc;

% Adjustable pameters
p.N = 2;   % Number of robots
p.a = 0.05 ; % Distance from wheel axle to spring attachment point
p.f2v = 1  ; % Velocity (m/s)   for every Newton of Force
p.t2h = 1  ; % Yaw rate (rad/s) for every Newton meter of Torque


% Fixed Constants
p.n = 3; % Number of states of one robot. The state is the X position, the Y position, and the angle Theta
p.m = 2; % Number of control inputs of one robot: The velocity and its yaw rate (

% Simulation time settings
t.end      = 100;                        % End of the simulation
t.interval = 0.1;                      % Sampling time
t.time     = 0:t.interval:t.end;       % Time vector with all the time instants
t.samples  = length(t.time);           % Number of time samples

% History of all the states and controls at all the time samples
r = zeros(p.N*p.n,t.samples);
v = zeros(p.N*p.m,t.samples);

% Initial conditions
r0 = (rand(p.N*p.n,1)-0.5)*100; % Random intial conditions
r(:,1) = r0;          % Put the initial conditions in the state matrix at time 0

%% Run the simulation
for k = 1:t.samples
    
    % Current state
    r_now = r(:,k);

    % Nett force vectors acting on all of the robots (Make a generic control law here.)
    F_now = ones(p.N*p.m);
    
    % Calculate the control signals
    v_now = force2velocity(F_now,r_now,p);   
    
    % Get the rate of state change, given the current state and the control signal
    r_dot_now = vehicle_state_change(r_now, v_now, p);
    
    % Euler integration step
    r_next = r_now + t.interval*r_dot_now;
    
    % Store the next state, if there is one
    if(k + 1 <= t.samples)
       r(:,k+1) = r_next; 
    end
end

%% Plot results
figure;hold on;
axis equal
lims = 100;
xlim([-lims lims])
ylim([-lims lims])

for i = 1:p.N
    
   % Get x and y trajectory for the i'th robot 
   [r_index, v_index] = array_index(i,p);
   x = r(r_index(1),:);
   y = r(r_index(2),:);
   
   % Add plot to the figure
   h = plot(x,y);
   
   % Plot starting point as a marker.
   plot(x(1),y(1),'x','Color',h.Color);
   
end



