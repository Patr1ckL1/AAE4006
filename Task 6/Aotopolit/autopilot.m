function y = autopilot(uu, P)
    % ---------------------------------------------------------    
    % extract states
    pn       = uu(1);  % inertial North position
    pe       = uu(2);  % inertial East position
    h        = -uu(3); % altitude
    u        = uu(4);  % velocity component in ib
    v        = uu(5);  % velocity component in jb
    w        = uu(6);  % velocity component in kb
    phi      = uu(7);  % roll angle
    theta    = uu(8);  % pitch angle
    psi      = uu(9);  % yaw angle 
    p        = uu(10); % body frame roll rate
    q        = uu(11); % body frame pitch rate
    r        = uu(12); % body frame yaw rate

    % extract airdata including Va, alpha, beta, w_n, w_e, w_d
    Va       = uu(13); % airspeed magnitude
    alpha    = uu(14); % angle of attack
    beta     = uu(15); % sideslip angle
    wn       = uu(16); % wind North
    we       = uu(17); % wind East
    wd       = uu(18); % wind Down
    
    % extract reference signals
    Va_c     = uu(19);  % commanded airspeed (m/s)
    h_c      = uu(20);  % commanded altitude (m)
    chi_c    = uu(21);  % commanded course (rad)

    t        = uu(22);  % time
    % ---------------------------------------------------------

    % compute course angle chi
    % define transformation matrix from body frame to vehicle frame
    R_roll = [...
          1, 0, 0;...
          0, cos(-phi), sin(-phi);...
          0, -sin(-phi), cos(-phi)];
    R_pitch = [...
          cos(-theta), 0, -sin(-theta);...
          0, 1, 0;...
          sin(-theta), 0, cos(-theta)];
    R_yaw = [...
          cos(-psi), sin(-psi), 0;...
          -sin(-psi), cos(-psi), 0;...
          0, 0, 1];
    R_b_v = R_yaw*R_pitch*R_roll;
    % compute airspeed vector in inertial frame
    V_a = R_b_v*[u,v,w]';
    % compute ground speed vector in inertial frame
    V_g = V_a + [wn,we,wd]';
    % compute course angle
    chi = atan2(V_g(2), V_g(1));

    %----------------------------------------------------------
    % lateral autopilot
   
    % control command of delta_a
    if t == 0
        phi_c = course_hold(chi_c, chi, 1, P);
    else
        phi_c = course_hold(chi_c, chi, 0, P);  
    end
    delta_a = roll_hold(phi_c, phi, p, P);
    
    % control command of delta_r
    if t == 0
        delta_r = sideslip_hold(0, beta, 1, P);
    else
        delta_r = sideslip_hold(0, beta, 0, P);  
    end

    %----------------------------------------------------------
    % longitudinal autopilot
    
    % define persistent variable for state of altitude
    persistent altitude_state;
    % persistent initialize_integrator;
    % initialize persistent variable
    if h <= P.altitude_take_off_zone    
        altitude_statet = 1;
    elseif h <= h_c-P.altitude_hold_zone
        altitude_statet = 2;
    elseif h >= h_c+P.altitude_hold_zone
        altitude_statet = 3;
    else
        altitude_statet = 4;
    end
    if t == 0
        initialize_integrator = 1;
    elseif altitude_state ~= altitude_statet
        initialize_integrator = 1;
    else
        initialize_integrator = 0;
    end
    altitude_state = altitude_statet;
    
    % implement state machine
    switch altitude_state
        case 1  % in take-off zone
            delta_t = 0.4; % a "large" throttle, should be adjusted with the flight request
            theta_c = P.climb_pitch;
        case 2  % climb zone
            delta_t = 0.4; % a "large" throttle, should be adjusted with the flight request
            theta_c = airspeed_with_pitch_hold(Va_c, Va, initialize_integrator, P);    
        case 3  % descend zone
            delta_t = 0; % a "small" throttle, should be adjusted with the flight request
            theta_c = airspeed_with_pitch_hold(Va_c, Va, initialize_integrator, P);
        case 4  % altitude hold zone
            delta_t = airspeed_with_throttle_hold(Va_c, Va, initialize_integrator, P);
            theta_c = altitude_hold(h_c, h, initialize_integrator, P);
    end
    delta_e = pitch_hold(theta_c, theta, q, P);

    % control outputs
    delta = [delta_e; delta_a; delta_r; delta_t];

    % commanded (desired) states
    x_command = [...
        0;...                    % pn, no desired value
        0;...                    % pe, no desired value
        h_c;...                  % h
        Va_c;...                 % Va
        0;...                    % alpha, no desired value 
        0;...                    % beta
        phi_c;...                % phi
        theta_c;...              % theta
        chi_c;...                % chi
        0;...                    % p, no desired value
        0;...                    % q, no desired value
        0;...                    % r, no desired value
        ];
    % create outputs        
    y = [delta; x_command];
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Autopilot functions
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Controller 1 (PD)
% roll_with_aileron - regulate roll using aileron
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function delta_a = roll_hold(phi_c, phi, p, P)
    error = phi_c-phi;
    differentiator = p;
    kp = P.kp_phi;
    kd = P.kd_phi;
    delta_a = sat(kp*error-kd*differentiator,P.delta_a_max,-P.delta_a_max);
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Controller 2 (PI)
% course_with_roll - regulate course angle using the roll angle
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function phi_c = course_hold(chi_c, chi, flag, P)
    persistent integratorchi
    if flag == 1
        integratorchi = 0;
    end
    error = chi_c - chi;
    integratorchi = integratorchi+P.Ts*error;
    phi_c = sat(P.kp_chi*error+P.ki_chi*integratorchi,P.phi_max,-P.phi_max);
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Controller 3 (PI)
% sideslip_with_rudder - regulate sideslip angle using the rudder
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function delta_r = sideslip_hold(beta_c, beta, flag, P)
    persistent integratorbeta
    if flag == 1
        integratorbeta = 0;
    end
    error = beta_c - beta;
    integratorbeta = integratorbeta+P.Ts*error;
    delta_r = sat(-P.kp_beta*error-P.ki_beta*integratorbeta,P.delta_r_max,-P.delta_r_max);
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Controller 4 (PD)
% pitch_with_elevator
%   - regulate pitch using elevator
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function delta_e = pitch_hold(theta_c, theta, q, P)

end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Controller 5 (PI)
% altitude_with_pitch
%   - regulate altitude using pitch angle
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function  theta_c = altitude_hold(h_c, h, flag, P)
    persistent integratorh

end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Controller 6 (PI)
% airspeed_with_pitch
%   - regulate airspeed using pitch angle
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function theta_c = airspeed_with_pitch_hold(Va_c, Va, flag, P)
    persistent integratorV2

end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Controller 7 (PI)
% airspeed_with_throttle
%   - regulate airspeed using throttle
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function delta_t = airspeed_with_throttle_hold(Va_c, Va, flag, P)
    persistent integratorV

end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% sat
%   - saturation function
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function out = sat(in, up_limit, low_limit)
  if in > up_limit
      out = up_limit;
  elseif in < low_limit
      out = low_limit;
  else
      out = in;
  end
end
