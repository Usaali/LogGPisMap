
function[]= draw_robot(center_location,L,W,theta,rgb)
% center_location: x an y in m
% L,W: length and width in m
% theta: heading in rad
% rgb: draw and face colour
center1=center_location(1);
center2=center_location(2);
R= ([cos(theta), -sin(theta); sin(theta), cos(theta)]);
X=([-L/2, L/2, 3*L/4, L/2, -L/2]);
Y=([-W/2, -W/2, 0, W/2, W/2]);

for i=1:5
    T(:,i)=R*[X(i); Y(i)];
end

x_lower_left=center1+T(1,1);
x_lower_right=center1+T(1,2);
x_peak=center1+T(1,3);
x_upper_right=center1+T(1,4);
x_upper_left=center1+T(1,5);

y_lower_left=center2+T(2,1);
y_lower_right=center2+T(2,2);
y_peak=center2+T(2,3);
y_upper_right=center2+T(2,4);
y_upper_left=center2+T(2,5);

x_coor=[x_lower_left x_lower_right x_peak x_upper_right x_upper_left];
y_coor=[y_lower_left y_lower_right y_peak y_upper_right y_upper_left];

patch('Vertices',[x_coor; y_coor]','Faces',[1 2 3 4 5],'Edgecolor','r','Facecolor',rgb,'Linewidth',1);
axis equal;
end