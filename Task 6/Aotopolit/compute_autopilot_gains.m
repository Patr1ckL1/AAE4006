
UAV.altitude_take_off_zone = 15; % meter
UAV.altitude_hold_zone = 10; % meter
UAV.climb_pitch = 20*pi/180; % radian
UAV.theta_max = 30*pi/180; % radian
UAV.phi_max = 30*pi/180; % radian

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% change the parameters of the aircraft if you are interested to study the
% influence of a certain parameter on the flight of the aircraft

% physical parameters of aircraft
UAV.mass = 20;
UAV.Jx   = 0.8244;
UAV.Jy   = 1.135;
UAV.Jz   = 1.759;
UAV.Jxz  = .1204;
% aerodynamic coefficients
UAV.S_wing        = 0.55;
UAV.b             = 2.8956;
UAV.c             = 0.18994;
UAV.S_prop        = 0.2027;
UAV.rho           = 1.2682;
UAV.k_motor       = 80;
UAV.k_T_P         = 0;
UAV.k_Omega       = 0;
UAV.e             = 0.9;

UAV.C_L_0         = 0.28;
UAV.C_L_alpha     = 3.45;
UAV.C_L_q         = 0.0;
UAV.C_L_delta_e   = -0.36;
UAV.C_D_0         = 0.03;
UAV.C_D_alpha     = 0.30;
UAV.C_D_p         = 0.0437;
UAV.C_D_q         = 0.0;
UAV.C_D_delta_e   = 0.0;
UAV.C_m_0         = -0.02338;
UAV.C_m_alpha     = -0.38;
UAV.C_m_q         = -3.6;
UAV.C_m_delta_e   = -0.5;
UAV.C_Y_0         = 0.0;
UAV.C_Y_beta      = -0.98;
UAV.C_Y_p         = 0.0;
UAV.C_Y_r         = 0.0;
UAV.C_Y_delta_a   = 0.0;
UAV.C_Y_delta_r   = -0.17;
UAV.C_ell_0       = 0.0;
UAV.C_ell_beta    = -0.12;
UAV.C_ell_p       = -0.26;
UAV.C_ell_r       = 0.14;
UAV.C_ell_delta_a = 0.08;
UAV.C_ell_delta_r = 0.105;
UAV.C_n_0         = 0.0;
UAV.C_n_beta      = 0.25;
UAV.C_n_p         = 0.022;
UAV.C_n_r         = -0.35;
UAV.C_n_delta_a   = 0.06;
UAV.C_n_delta_r   = -0.032;
UAV.C_prop        = 1.0;
UAV.M             = 50;
UAV.epsilon       = 0.1592;
UAV.alpha0        = 0.4712;

% wind parameters
UAV.wind_n = 0;
UAV.wind_e = 0;
UAV.wind_d = 0;
UAV.L_u = 200;
UAV.L_v = 200;
UAV.L_w = 50;
UAV.sigma_u = 1; 
UAV.sigma_v = 1;
UAV.sigma_w = 1;


% compute trim conditions
% initial airspeed, flight path angle, and turning radius
UAV.Va0 = 35;
gamma = 0;
R = inf;

% autopilot sample rate
UAV.Ts = 0.01;

% first cut at initial conditions
UAV.pn0    = 0;  % initial North position
UAV.pe0    = 0;  % initial East position
UAV.pd0    = 0;  % initial Down position 
UAV.u0     = UAV.Va0; % initial velocity along body x-axis
UAV.v0     = 0;  % initial velocity along body y-axis
UAV.w0     = 0;  % initial velocity along body z-axis
UAV.phi0   = 0;  % initial roll angle
UAV.theta0 = 0;  % initial pitch angle
UAV.psi0   = 0;  % initial yaw angle
UAV.p0     = 0;  % initial body frame roll rate
UAV.q0     = 0;  % initial body frame pitch rate
UAV.r0     = 0;  % initial body frame yaw rate

% run trim commands
[x_trim, u_trim] = compute_trim('uavsim_trim', UAV.Va0, gamma, R);
UAV.u_trim = u_trim;
UAV.x_trim = x_trim;

% set initial conditions to trim conditions
% initial conditions
UAV.pn0    = 0;  % initial North position
UAV.pe0    = 0;  % initial East position
UAV.pd0    = 0;  % initial Down position 
UAV.u0     = x_trim(4);  % initial velocity along body x-axis
UAV.v0     = x_trim(5);  % initial velocity along body y-axis
UAV.w0     = x_trim(6);  % initial velocity along body z-axis
UAV.phi0   = x_trim(7);  % initial roll angle
UAV.theta0 = x_trim(8);  % initial pitch angle
UAV.psi0   = x_trim(9);  % initial yaw angle
UAV.p0     = x_trim(10); % initial body frame roll rate
UAV.q0     = x_trim(11); % initial body frame pitch rate
UAV.r0     = x_trim(12); % initial body frame yaw rate

UAV.u_r = x_trim(4);
UAV.v_r = x_trim(5);
UAV.w_r = x_trim(6);
UAV.theta_trim = x_trim(8);
UAV.delta_e_trim = u_trim(1);
UAV.delta_t_trim = u_trim(4);
UAV.Va_trim = sqrt(UAV.u_r^2+UAV.v_r^2+UAV.w_r^2);
UAV.alpha_trim = atan(UAV.w_r/UAV.u_r);

UAV.gamma = UAV.Jx*UAV.Jz-UAV.Jxz^2;
UAV.gam3 = UAV.Jz/UAV.gamma;
UAV.gam4 = UAV.Jxz/UAV.gamma;

UAV.C_p_p = UAV.gam3*UAV.C_ell_p+UAV.gam4*UAV.C_n_p;
UAV.C_p_delta_a = UAV.gam3*UAV.C_ell_delta_a+UAV.gam4*UAV.C_n_delta_a;

UAV.a_phi1 = -0.5*UAV.rho*UAV.Va0^2*UAV.S_wing*UAV.b*UAV.C_p_p*UAV.b/2/UAV.Va0;
UAV.a_phi2 = 0.5*UAV.rho*UAV.Va0*2*UAV.S_wing*UAV.C_p_delta_a;
UAV.a_beta1 = -(UAV.rho*UAV.Va0*UAV.S_wing)/(2*UAV.mass)*UAV.C_Y_beta;
UAV.a_beta2 = (UAV.rho*UAV.Va0*UAV.S_wing)/(2*UAV.mass)*UAV.C_Y_delta_r;
UAV.a_theta1 = -(UAV.rho*UAV.Va0^2*UAV.c*UAV.S_wing)/(2*UAV.Jy)*UAV.C_m_q*UAV.c/2/UAV.Va0;
UAV.a_theta2 = -(UAV.rho*UAV.Va0^2*UAV.c*UAV.S_wing)/(2*UAV.Jy)*UAV.C_m_alpha;
UAV.a_theta3 = -(UAV.rho*UAV.Va0^2*UAV.c*UAV.S_wing)/(2*UAV.Jy)*UAV.C_m_delta_e;
UAV.a_V1 = UAV.rho*UAV.Va0*UAV.S_wing/UAV.mass*(UAV.C_D_0+UAV.C_D_alpha*UAV.alpha_trim+UAV.C_D_delta_e*UAV.delta_e_trim)...
    + UAV.rho*UAV.S_prop/UAV.mass*UAV.C_prop*UAV.Va0;
UAV.a_V2 = UAV.rho*UAV.S_prop/UAV.mass*UAV.C_prop*UAV.k_motor^2*UAV.delta_t_trim;
UAV.a_V3 = UAV.gravity*cos(UAV.theta_trim-UAV.alpha_trim);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Work on the designing of PID parameters
% lateral
% roll
UAV.delta_a_max = 45*pi/180;
UAV.e_phi_max = 15*pi/180;
UAV.kp_phi = UAV.delta_a_max/UAV.e_phi_max*sign(UAV.a_phi2);
UAV.omega_n_phi = sqrt(abs(UAV.a_phi2)*UAV.delta_a_max/UAV.e_phi_max);
UAV.zeta_phi = 3;
UAV.kd_phi = (2*UAV.zeta_phi*UAV.omega_n_phi-UAV.a_phi1)/UAV.a_phi2;

% course
UAV.W_x = 10; %bandwidth separation for roll and course
UAV.omega_n_chi = UAV.omega_n_phi/UAV.W_x;
UAV.zeta_chi = 1.2;
UAV.kp_chi = 2*UAV.zeta_chi*UAV.omega_n_chi*UAV.Va0/UAV.gravity;
UAV.ki_chi = UAV.omega_n_chi^2*UAV.Va0/UAV.gravity;

% sideslip
UAV.e_beta_max = 20*pi/180;
UAV.zeta_beta = 10;
UAV.delta_r_max = 10*pi/180;
UAV.kp_beta = UAV.delta_r_max/UAV.e_beta_max*sign(UAV.a_beta2);
UAV.ki_beta = 1/UAV.a_beta2*(UAV.a_beta1+UAV.a_beta2*UAV.kp_beta)^2/(2*UAV.zeta_beta)^2;


% longitudinal 
% pitch
UAV.delta_e_max = 45*pi/180;
UAV.e_theta_max = 10*pi/180;
UAV.kp_theta = ;
UAV.omega_n_theta = ;
UAV.zeta_theta = ;
UAV.kd_theta = ;
UAV.K_theta_DC = ;

% altitude
UAV.W_h = ;
UAV.omega_n_h = ;
UAV.zeta_h = ;
UAV.ki_h = ;
UAV.kp_h = ;

% airspeed with Pitch
UAV.W_V2 = ;
UAV.omega_n_V2 = ;
UAV.zeta_V2 = ;
UAV.ki_V2 = ;
UAV.kp_V2 = ;

% airspeed with throttle
UAV.omega_n_V = ;
UAV.zeta_V = ;
UAV.ki_V = ;
UAV.kp_V = ;