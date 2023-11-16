% TK: Tobias Kaupp
% This is a fork of script demo_gpisMap.m
% It includes generating an animated gif


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

clearvars
close all

addpath(genpath('./util'));
addpath('./plot_scripts');
addpath('../mex');

%
% TK: user parameters for plotting
%
makeGif=0;
plotDistanceField=0;
plotGradientsNonNormalised=0;
plotGradientsNormalised=0;
plotGradientsScaled=1;

% TK: we get 
%  poses: x, y, orientation
%  thetas: -135 to 135 degrees (but in rad)
%  ranges: range observations for angles -135 to 135 degrees
load('../data/2D/gazebo1.mat');

% test points (regular samples for visualization)
xmin = -5;
xmax = 20;
ymin = -15;
ymax = 5;
test_intv = 0.1;
[xg, yg] = meshgrid((xmin+test_intv):test_intv:(xmax-test_intv), (ymin+test_intv):test_intv:(ymax-test_intv));
xtest = single([xg(:)'; yg(:)']);

skip = 50;
initframe = 101; % first 100 frames are almost static...
lastframe = (floor((size(poses,1)-initframe)/skip))*skip+initframe;

% TK: just one frame
% nframe = 101;

for nframe = initframe:skip:lastframe

    % pose
    tr = poses(nframe,1:2)';
    phi = poses(nframe,3);
    Rot = [cos(phi) -sin(phi); sin(phi) cos(phi)];

    % update
    % TK
    disp('Calling mexGPisMap:update')
    tic
    % TK: we send LIDAR range observations and robot pose
    
    mexGPisMap('update',single(thetas),...
                        single(ranges(nframe,:)'),...
                        single([tr; reshape(Rot,[],1)]));
    toc
    % test visualization
    if  1 %nframe == lastframe % set the condition to 1 to visualize every update
        visualize_gpisMapRobotGradients
        if makeGif
            if nframe == initframe
               display('nframe==initframe');
               gif('output.gif', 'DelayTime',0.4)
            else
               display('nframe != initframe');
               gif
            end
        end
    end

    % pause if needed
    %disp('Press a button to continue')
    %pause

end

% delete all resources
mexGPisMap('reset');
