% Log-GPIS - Faithful Euclidean distance field from Log-Gaussian Process Implicit Surfaces
% https://github.com/LanWu076/LogGPisMap
% Authors: Lan Wu, Ki Myung Brian Lee
% Email: iriswu076@gmail.com

% GPisMap - Online Continuous Mapping using Gaussian Process Implicit Surfaces
% https://github.com/leebhoram/GPisMap
% Authors: Bhoram Lee <bhoram.lee@gmail.com>

% This program is free software; you can redistribute it and/or modify
% it under the terms of the GNU General Public License v3 as published by
% the Free Software Foundation.
%
% This program is distributed in the hope that it will be useful, but WITHOUT
% ANY WARRANTY; without even the implied warranty of any FITNESS FOR A
% PARTICULAR PURPOSE. See the GNU General Public License v3 for more details.
%
% You should have received a copy of the GNU General Public License v3
% along with this program; if not, you can access it online at
% http://www.gnu.org/licenses/gpl-3.0.html.
%

disp('drawing...');

gp_bias = 0;
w = 10;
sensor_offset = [0.08; 0];
res = mexGPisMap('test',xtest);

% new mean
res(1,:) = -(1/w)*log(res(1,:));
fval = res(1,:) + gp_bias;

% two surfaces extraction
[~,temp] = isocontour(reshape(fval,size(xg)),0.1);

% plot two surfaces with normals
tx = ((xmax-xmin)*(temp(:,2)-0.5)/size(xg,2)) + xmin;
ty = ((ymax-ymin)*(temp(:,1)-0.5)/size(yg,1)) + ymin;
tepp = single([tx, ty])';
teppRes = mexGPisMap('test',tepp);
loc = tepp'; % Location

% normalised the normals
nx = -[teppRes(2,:)',teppRes(3,:)'];
nx = nx ./ sqrt(sum(nx.^2, 2));

% check the double surfaces
% figure;
% quiver(loc(:,1), loc(:,2), nx(:,1), nx(:,2)); hold on;
% axis([xmin xmax ymin ymax])
% axis equal;

% find local minimum as final surface points
locc = loc(:,:);
nxx = nx(:,:);
finalpoints = [];
for loop = 1 : numel(locc(:,1))
    raypoint = locc(loop,:);
    biaspoint = nxx(loop,:);
    x = [raypoint(1,1)-1*biaspoint(1,1);raypoint(1,1)]; 
    v = [raypoint(1,2)-1*biaspoint(1,2);raypoint(1,2)]; 

    if((raypoint(1,1)-1*biaspoint(1,1))<raypoint(1,1))
        xq = linspace((raypoint(1,1)-1*biaspoint(1,1)),raypoint(1,1),30);
    else
        xq = linspace(raypoint(1,1),(raypoint(1,1)-1*biaspoint(1,1)),30);
    end
    vq1 = interp1(x,v,xq);

    %plot(x,v,'o',xq',vq1','.');
    rayline = single([xq; vq1]);
    test_rayline = mexGPisMap('test',rayline);
    test_rayline(1,:) = -(1/w)*log(test_rayline(1,:));
    [~,IndexMin] = min(test_rayline(1,:));
    finalpoints = [finalpoints,rayline(:,IndexMin)];
end

% we lost the sign, so need the mask to recover it
maskpoints = [];
for nframe1 = 200:300:lastframe
    
valid = find((ranges(nframe1,:)'<3e1) & (ranges(nframe1,:)'>2e-1) & (~isinf(ranges(nframe1,:)')));
XY = polar2xy(thetas(valid),ranges(nframe1,(valid))');
XY(1,:) = XY(1,:) + sensor_offset(1);

tr = poses(nframe1,1:2)';
phi = poses(nframe1,3);
Rot = [cos(phi) -sin(phi); sin(phi) cos(phi)];
XY_ref = Rot*XY + tr;

for t=1:numel(valid)
    maskPx = linspace(tr(1),XY_ref(1,t),70);
    maskPy = interp1([tr(1);XY_ref(1,t)],[tr(2);XY_ref(2,t)],maskPx);
    maskpoints = [maskpoints;[maskPx;maskPy]'];
end
end
% plot(maskpoints(:,1), maskpoints(:,2),'y*'); hold on;

% generate the mask for recovering the sign
mask = size(xtest,2);
xtest1 = xtest';

[k,dist] = knnsearch(maskpoints,xtest1);
valid1 = find(dist >= 0.15);
valid2 = find(dist < 0.15);

mask(valid2) = 1;
mask(valid1) = -1;
mask = mask';

% plot the results
figure;
see1 = reshape(fval'.*mask,size(xg));
h = pcolor(xg,yg,reshape(fval'.*mask,size(xg))); hold on;
set(h,'EdgeColor','none');
finalpoints = finalpoints';
plot(finalpoints(:,1), finalpoints(:,2),'r.','MarkerSize',5); hold on;
axis equal;
axis on;
colorbar;
lim = [-4 4];
caxis(lim);  
hold on;