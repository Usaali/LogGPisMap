% Adapted from Rafal Szczepanski
% https://au.mathworks.com/matlabcentral/fileexchange/126455-artificial-potential-field-path-planning-algorithm?s_tid=prof_contriblnk


clear all; close all; clc;

% Initial position and orientation 
x = -0.5;
y = 0.5;
% x = -0.5;
% y = 0.3;
theta = 0;

% Goal position
x_goal = 3.5;
y_goal = 2.75;
% x_goal = 1.5;
% y_goal = 0.5;
position_accuracy = 0.05;

% APF parameters
zeta = 1.1547;
eta = 0.0732;
dstar = 0.3;
Qstar = 0.75;

% Parameters related to kinematic model
error_theta_max = deg2rad(45);
v_max = 0.2;
Kp_omega = 1.5;
omega_max = 0.5*pi; 

% Generate obstacles
obst1_points = [ linspace(1.5,1.5,100) linspace(1.5,2,100) linspace(2,2,100)    linspace(2,1.5,100) 
                 linspace(1.5,2,100)   linspace(2,2,100)   linspace(2,1.5,100)  linspace(1.5,1.5,100) ];
obst1_points(1,:) = obst1_points(1,:) - 1;
obst1_points(2,:) = obst1_points(2,:) - 1;
obst2_points = [ 2+sin(linspace(0,pi/2,100))     linspace(3,3,100)    linspace(3,2,100)           
                 2.5+cos(linspace(0,pi/2,100))   linspace(2.5,3.5,100)  linspace(3.5,3.5,100)    ];
obst2_points(2,:) = obst2_points(2,:) - 1.5;

figure(1); 

t = 1;
dT = 0.1;
t_max = 1000;
X = zeros(1,t_max);
Y = zeros(1,t_max);
X(1) = x;
Y(1) = y;

NablaU_Att = zeros(2,t_max);
NablaU_Rep = zeros(2,t_max);

while norm([x_goal y_goal] - [x y]) > position_accuracy || t > t_max   
    
    % Calculate Attractive Potential
    if norm([x y]-[x_goal y_goal]) <= dstar
        nablaU_att =  zeta*([x y]-[x_goal y_goal]);
    else 
        nablaU_att = dstar/norm([x y]-[x_goal y_goal]) * zeta*([x y]-[x_goal y_goal]);
    end
    NablaU_Att(:,t) = nablaU_att;
    
    % Find the minimum distance from the obstacle
    [obst1_idx, obst1_dist] = dsearchn(obst1_points', [x y]);
    [obst2_idx, obst2_dist] = dsearchn(obst2_points', [x y]);
    
    % Calculate Repulsive Potential
    nablaU_rep = [0 0];
    if obst1_dist <= Qstar     
        nablaU_rep = nablaU_rep + (eta*(1/Qstar - 1/obst1_dist) * 1/obst1_dist^2)*([x y] - [obst1_points(1,obst1_idx)  obst1_points(2,obst1_idx)]);
    end
    if obst2_dist <= Qstar  && ~inpolygon(x,y,obst2_points(1,:),obst2_points(2,:))          
        nablaU_rep = nablaU_rep + (eta*(1/Qstar - 1/obst2_dist) * 1/obst2_dist^2)*([x y] - [obst2_points(1,obst2_idx)  obst2_points(2,obst2_idx)]);
    end
    NablaU_Rep(:,t) = nablaU_rep;
    
    % Calculate final potential
    nablaU = nablaU_att+nablaU_rep;
    
    % Calculate reference value of linear velocity (v_ref) and orientation (theta_ref)
    theta_ref = atan2(-nablaU(2), -nablaU(1));
    error_theta = theta_ref - theta;
    if abs(error_theta) <= error_theta_max
        alpha = (error_theta_max - abs(error_theta)) / error_theta_max;
        v_ref = min( alpha*norm(-nablaU), v_max );
    else
        v_ref = 0;
    end
    
    % Simple kinematic mobile robot model
    % Omitted dynamics.
    omega_ref = Kp_omega * error_theta;
    omega_ref = min( max(omega_ref, -omega_max), omega_max);
    theta = theta + omega_ref * dT;
    x = x + v_ref*cos(theta) * dT;
    y = y + v_ref*sin(theta) * dT;
    t = t + 1;
    
    % Archive and plot it
    X(t) = x;
    Y(t) = y;
    %cla;
    daspect([1 1 1]); 
    xlim([-1,4]);  ylim([-1 3]);
    box on; hold on;
    plot(obst1_points(1,:), obst1_points(2,:), '-r');
    plot(obst2_points(1,:), obst2_points(2,:), '-r');
    plot(x_goal, y_goal, 'ob');
    %plot(X(1:t), Y(1:t), '-b'); % Plot traveled path
    %plot([x x+0.2*cos(theta_ref)], [y y+0.2*sin(theta_ref)], '-g'); % Plot reference orientation of the robot
    %plot([x x+0.2*cos(theta)], [y y+0.2*sin(theta)], '-r'); % Plot orientation of the robot
    
    % show the repulsive vectors
    quiver(x,y,nablaU_att(1),nablaU_att(2),0,'b');
    quiver(x,y,nablaU_rep(1),nablaU_rep(2),0,'r');
    quiver(x,y,nablaU(1),nablaU(2),0,'g');
    
    drawnow;
    pause(dT);
end

travelTimeSec = t*dT; % scale from itetations to [s]
disp("Travel time: " + travelTimeSec);

NablaU_Rep(:,t+1:end) = [];
NablaU_Att(:,t+1:end) = [];

% figure(2);
% hold on;
% plot(NablaU_Rep(1,:), 'r*')
% plot(NablaU_Rep(2,:), 'b*')
% magRep = sqrt( NablaU_Rep(1,:).^2 + NablaU_Rep(2,:).^2 );
% plot(magRep, 'gd');
% title('Repulsive');
% 
% figure(3);
% hold on;
% plot(NablaU_Att(1,:), 'r*')
% plot(NablaU_Att(2,:), 'b*')
% magAtt = sqrt( NablaU_Att(1,:).^2 + NablaU_Att(2,:).^2 );
% plot(magAtt, 'gd');
% title('Attractive')
