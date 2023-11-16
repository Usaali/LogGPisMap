clear all;
close all;

load('results\data_full_run.mat');

% Start and end goal
startPose = [3 0.5 0];
endPosition = [5 -4.5];
endPositionAccuracy = 0.1;

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

figure(99)
hold on;
quiver(xg(idx),yg(idx),gradNormXScaled,gradNormYScaled);
%draw_robot(poses(nframe,1:2),L,W,poses(nframe,3),rgb)
title('Scaled Gradients')
axis equal;

% xmin=min(startPose(1), endPosition(1))-0.5;
% xmax=max(startPose(1), endPosition(1))+0.5;
% ymin=min(startPose(2), endPosition(2))-0.5;
% ymax=max(startPose(2), endPosition(2))+0.5;
xlim([xmin xmax])
ylim([ymin ymax])

plot(startPose(1), startPose(2), 'ok', 'MarkerFaceColor', 'k', 'MarkerSize', 5);
plot(endPosition(1), endPosition(2), 'or', 'MarkerFaceColor', 'r', 'MarkerSize', 5);

t = 1;
dT = 0.1;
t_max = 1000;
X = zeros(1,t_max);
Y = zeros(1,t_max);
x = startPose(1);
y = startPose(2);
theta = startPose(3);
X(1) = x;
Y(1) = y;

% for searching the closest gradient
xMeshLocations = xg(1,:);
yMeshLocations = yg(:,2);
gradNormXScaledReshaped = reshape(gradNormXScaled, size(xg));
gradNormYScaledReshaped = reshape(gradNormYScaled, size(xg));
repScaleFactor = 0.15;

while norm(endPosition - [x y]) > endPositionAccuracy || t > t_max
    % Calculate Attractive Potential
    if norm([x y] - endPosition) <= dstar
        nablaU_att =  zeta*([x y] - endPosition);
    else 
        nablaU_att = dstar/norm( [x y] - endPosition) * zeta*([x y] - endPosition);
    end
    
    % Calculate Repulsive Potential
    nablaU_rep = [0 0];
    
    % find closest gradient to x and y
    [dx, indX] = min( abs( xMeshLocations-x) );
    [dy, indY] = min( abs( yMeshLocations-y) );
    % negative since our gradients are the other way round
    nablaU_rep = - [ gradNormXScaledReshaped(indY,indX) gradNormYScaledReshaped(indY,indX) ] .* repScaleFactor;
    
    % Calculate final potential
    nablaU = nablaU_att+nablaU_rep;
    
    
    % plot vectors
%     if t>1
%         delete(q1); delete(q2); delete (q3);
%     end
    
    hold on;
    q1=quiver(x,y,nablaU_att(1),nablaU_att(2),0,'b');
    q2=quiver(x,y,nablaU_rep(1),nablaU_rep(2),0,'r');
    q3=quiver(x,y,nablaU(1),nablaU(2),0,'g');
    
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

    %plot(X(1:t), Y(1:t), '-k'); % Plot traveled path
    %plot([x x+0.2*cos(theta_ref)], [y y+0.2*sin(theta_ref)], '-g'); % Plot reference orientation of the robot
    %plot([x x+0.2*cos(theta)], [y y+0.2*sin(theta)], '-r'); % Plot orientation of the robot

    drawnow;
    pause(dT);
    
end

travelTime = t*dT; % scale from iterations to [s]
disp("Travel time: " + travelTime);
