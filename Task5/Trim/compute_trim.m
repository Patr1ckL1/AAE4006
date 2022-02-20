% This file is to compute trim conditions using 'uavsim_trim.slx'.
% So, have this simulink model 'uavsim_trim.slx' ready to use this file.
Va = 25;            % desired airspeed magnitude
gamma = 0*pi/180;   % desired flight path angle (radians)
R     = 300;        % desired radius (m) - use (+) for right handed orbit, 
                    %                          (-) for left handed orbit
UAV.Va0 = Va;

% set initial conditions 
x0 = [0; 0; 0; Va; 0; 0; 0; gamma; 0; 0; 0; 0];
% specify which states to hold equal to the initial conditions
ix = [];

% specify initial inputs 
u0 = [...
    0;... % 1 - delta_e
    0;... % 2 - delta_a
    0;... % 3 - delta_r
    1;... % 4 - delta_t
    ];
% specify which inputs to hold constant
iu = [];

% define constant outputs
y0 = [...
    Va;...       % 1 - Va
    0;...        % 2 - alpha
    0;...        % 3 - beta
    ];
% specify which outputs to hold constant
iy = [1,3];

% define constant derivatives
dx0 = [0; 0; -Va*sin(gamma); 0; 0; 0; 0; 0; Va*cos(gamma)/R; 0; 0; 0];

% specify which derivaties to hold constant in trim algorithm
idx = [3; 4; 5; 6; 7; 8; 9; 10; 11; 12];

% compute trim conditions
[x_trim,u_trim,y_trim,dx_trim] = trim('uavsim_trim',x0,u0,y0,ix,iu,iy,dx0,idx);

% check to make sure that the linearization worked (should be small)
norm(dx_trim(3:end)-dx0(3:end))

% assign the trim conditions as the initial conditions
% initial conditions
UAV.pn0    = 0;           % initial North position
UAV.pe0    = 0;           % initial East position
UAV.pd0    = -100;        % initial Down position (negative altitude)
UAV.u0     = x_trim(4);   % initial velocity along body x-axis
UAV.v0     = x_trim(5);   % initial velocity along body y-axis
UAV.w0     = x_trim(6);   % initial velocity along body z-axis
UAV.phi0   = x_trim(7);   % initial roll angle
UAV.theta0 = x_trim(8);   % initial pitch angle
UAV.psi0   = x_trim(9);   % initial yaw angle
UAV.p0     = x_trim(10);  % initial body frame roll rate
UAV.q0     = x_trim(11);  % initial body frame pitch rate
UAV.r0     = x_trim(12);  % initial body frame yaw rate  