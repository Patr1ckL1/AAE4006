clear;clc;close all
% parameters of the aircraft model
fuse_l1=1.5; % center of mass to tip of fuselage
fuse_l2=1; % center of mass to widest part of fuselage
fuse_l3=5; % center of mass to back of fuselage
fuse_h=1;
fuse_w=1;
wing_l=2; % length of wing along fuselage
wing_w=6; % wingspan
tail_h=1;
tailwing_l=1;
tailwing_w=3;

% Define the vertices (physical location of vertices)
  V = []; % 16 vertices totally
% define surfaces as a list of numbered vertices
  F = [];
% define colors for each face    
  myred = [1, 0, 0];
  mygreen = [0, 1, 0];
  myblue = [0, 0, 1];
  myyellow = [1, 1, 0];
  mycyan = [0, 1, 1];

  colors = [...
    mygreen;...  % front top
    mycyan;...  % front left
    myblue;...  % front bottom
    mycyan;...  % front right 
    mygreen;...  % main top 
    mycyan;...  % main left
    myblue;...  % main bottom 
    mycyan;...  % main right
    myred;...  % wings
    myred;...  % tailwing
    myyellow;...  % tailfin
    ];
% transform vertices from NED to XYZ (for matlab rendering)
R = [0, 1, 0;...
      1, 0, 0;...
      0, 0, -1];
V1 = V*R;
handle = patch('Vertices', V1, 'Faces', F,'FaceVertexCData',colors,'FaceColor','flat');
title('Spacecraft')
xlabel('East')
ylabel('North')
zlabel('-Down')
hold on