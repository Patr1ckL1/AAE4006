% clear;clc;close all
%% Put the code you developed last week about drawing an aircraft below

%% Machine-user interface to ask for the angles of phi, theta and psi
prompt = 'Set your desired value of phi: ';
phi = input(prompt);
prompt = 'Set your desired value of theta: ';
theta = input(prompt);
prompt = 'Set your desired value of psi: ';
psi = input(prompt);
%% rotation matrix
% Implement the three rotation matrices 
R_roll = [];
R_pitch = [];
R_yaw = [];
V = (R_roll* R_pitch*R_yaw * V')';
%% transform vertices from NED to XYZ (for matlab rendering)
R = [...
      0, 1, 0;...
      1, 0, 0;...
      0, 0, -1;...
      ];
V = V*R;
%% redraw 
set(handle,'Vertices',V,'Faces',F);