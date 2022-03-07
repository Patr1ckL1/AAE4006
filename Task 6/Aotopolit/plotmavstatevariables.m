function plotmavstatevariables(uu)
    % process inputs to function
    pn          = uu(1);             % North position (meter)
    pe          = uu(2);             % East position (meter)
    h           = -uu(3);            % altitude (meter)
    u           = uu(4);             % body velocity along x-axis (meter/s)
    v           = uu(5);             % body velocity along y-axis (meter/s)
    w           = uu(6);             % body velocity along z-axis (meter/s)
    phi         = 180/pi*uu(7);      % roll angle (degree)   
    theta       = 180/pi*uu(8);      % pitch angle (degree)
    psi         = uu(9);             % yaw angle (radian)
    p           = 180/pi*uu(10);     % body angular rate along x-axis (radian/s)
    q           = 180/pi*uu(11);     % body angular rate along y-axis (radian/s)
    r           = 180/pi*uu(12);     % body angular rate along z-axis (radian/s)
    Va          = uu(13);            % airspeed (m/s)
    alpha       = 180/pi*uu(14);     % angle of attack (degree)
    beta        = 180/pi*uu(15);     % sideslip angle (degree)
    wn          = uu(16);            % wind in the North direction
    we          = uu(17);            % wind in the East direction
    wd          = uu(18);            % wind in the Down direction
    pn_c        = uu(19);            % commanded North position (meter)
    pe_c        = uu(20);            % commanded East position (meter)
    h_c         = uu(21);            % commanded altitude (meter)
    Va_c        = uu(22);            % commanded airspeed (meter/s)
    alpha_c     = 180/pi*uu(23);     % commanded angle of attack (degree)
    beta_c      = 180/pi*uu(24);     % commanded sideslip angle (degree)
    phi_c       = 180/pi*uu(25);     % commanded roll angle (degrees)   
    theta_c     = 180/pi*uu(26);     % commanded pitch angle (degree)
    chi_c       = 180/pi*uu(27);     % commanded course (degree)
    p_c         = 180/pi*uu(28);     % commanded body angular rate along x-axis (degree)
    q_c         = 180/pi*uu(29);     % commanded body angular rate along y-axis (degree)
    r_c         = 180/pi*uu(30);     % commanded body angular rate along z-axis (degree)
    
    delta_e     = 180/pi*uu(50);     % elevator angle (degrees)
    delta_a     = 180/pi*uu(51);     % aileron angle (degrees)
    delta_r     = 180/pi*uu(52);     % rudder angle (degrees)
    delta_t     = uu(53);            % throttle setting (unitless)
    t           = uu(54);            % simulation time
    
    % compute course angle
    chi = 180/pi*atan2(Va*sin(psi)+we, Va*cos(psi)+wn);

    % define persistent variables 
    persistent pn_handle
    persistent pe_handle
    persistent h_handle
    persistent Va_handle
    persistent alpha_handle
    persistent beta_handle
    persistent phi_handle
    persistent theta_handle
    persistent chi_handle
    persistent p_handle
    persistent q_handle
    persistent r_handle
    persistent delta_e_handle
    persistent delta_a_handle
    persistent delta_r_handle
    persistent delta_t_handle

    % first time function is called, initialize plot and persistent vars
    if t==0
        figure(2), clf

        subplot(8,2,1)
        hold on
        pn_handle = graph_y(t, pn,  [], 'b');
        ylabel('p_n')
        set(get(gca, 'YLabel'),'Rotation',0.0);
        
        subplot(8,2,2)
        hold on
        Va_handle = graph_y_yd(t, Va, Va_c, 'V_a', []);
        legend('Real','Reference')
        set(get(gca, 'YLabel'),'Rotation',0.0);
        
        subplot(8,2,3)
        hold on
        pe_handle = graph_y(t, pe,  [], 'b');
        ylabel('p_e')
        set(get(gca, 'YLabel'),'Rotation',0.0);
        
        subplot(8,2,4)
        hold on
        alpha_handle = graph_y(t, alpha,  [], 'b');
        ylabel('\alpha')
        set(get(gca, 'YLabel'),'Rotation',0.0);
        
        subplot(8,2,5)
        hold on
        h_handle = graph_y_yd(t, h, h_c, 'h', []);

        subplot(8,2,6)
        hold on
        beta_handle = graph_y_yd(t, beta, beta_c, '\beta', []);
        ylabel('\beta')
        set(get(gca, 'YLabel'),'Rotation',0.0);

        subplot(8,2,7)
        hold on
        phi_handle = graph_y(t, phi,  [], 'b');
        ylabel('\phi')
        set(get(gca, 'YLabel'),'Rotation',0.0);
        
        subplot(8,2,8)
        hold on
        p_handle = graph_y(t, p,  [], 'b');
        ylabel('p')
        set(get(gca, 'YLabel'),'Rotation',0.0);
        
        subplot(8,2,9)
        hold on
        theta_handle = graph_y(t, theta,  [], 'b');
        ylabel('\theta')
        set(get(gca, 'YLabel'),'Rotation',0.0);
        
        subplot(8,2,10)
        hold on
        q_handle = graph_y(t, q,  [], 'b');
        ylabel('q')
        set(get(gca, 'YLabel'),'Rotation',0.0);
        
        subplot(8,2,11)
        hold on        
        chi_handle = graph_y_yd(t, chi, chi_c, '\chi', []);
        
        subplot(8,2,12)
        hold on
        r_handle = graph_y(t, r,  [], 'b');
        ylabel('r')
        set(get(gca, 'YLabel'),'Rotation',0.0);
        
        subplot(8,2,13)
        hold on        
        delta_e_handle = graph_y(t, delta_e, [], 'b');
        ylabel('\delta_e')
        set(get(gca, 'YLabel'),'Rotation',0.0);
        
        subplot(8,2,14)
        hold on
        delta_a_handle = graph_y(t, delta_a, [], 'b');
        ylabel('\delta_a')
        set(get(gca, 'YLabel'),'Rotation',0.0);

        subplot(8,2,15)
        hold on
        delta_r_handle = graph_y(t, delta_r, [], 'b');
        ylabel('\delta_r')
        set(get(gca, 'YLabel'),'Rotation',0.0);
        
        subplot(8,2,16)
        hold on
        delta_t_handle = graph_y(t, delta_t, [], 'b');
        ylabel('\delta_t')
        set(get(gca, 'YLabel'),'Rotation',0.0);
        
    % at every other time step, redraw state variables
    else
       graph_y(t, pn, pn_handle);
       graph_y(t, pe, pe_handle);
       graph_y_yd(t, h, h_c, 'h', h_handle);
       graph_y_yd(t, Va, Va_c, 'V_a', Va_handle);
       graph_y(t, alpha, alpha_handle);
       graph_y_yd(t, beta, beta_c, '\beta', beta_handle);
       graph_y(t, phi, phi_handle);
       graph_y(t, theta, theta_handle);
       graph_y_yd(t, chi, chi_c, '\chi', chi_handle);
       graph_y(t, p, p_handle);
       graph_y(t, q, q_handle);
       graph_y(t, r, r_handle);
       graph_y(t, delta_e, delta_e_handle);
       graph_y(t, delta_a, delta_a_handle);
       graph_y(t, delta_r, delta_r_handle);
       graph_y(t, delta_t, delta_t_handle);
    end



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% graph y with lable mylabel
function handle = graph_y(t, y, handle, color)
  
  if isempty(handle)
    handle    = plot(t,y,color);
  else
    set(handle,'Xdata',[get(handle,'Xdata'),t]);
    set(handle,'Ydata',[get(handle,'Ydata'),y]);
    %drawnow
  end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% graph y and yd with lable mylabel
function handle = graph_y_yd(t, y, yd, lab, handle)
  
  if isempty(handle)
    handle(1)    = plot(t,y,'b');
    handle(2)    = plot(t,yd,'r--');
    ylabel(lab)
    set(get(gca, 'YLabel'),'Rotation',0.0);
  else
    set(handle(1),'Xdata',[get(handle(1),'Xdata'),t]);
    set(handle(1),'Ydata',[get(handle(1),'Ydata'),y]);
    set(handle(2),'Xdata',[get(handle(2),'Xdata'),t]);
    set(handle(2),'Ydata',[get(handle(2),'Ydata'),yd]);
    %drawnow
  end

