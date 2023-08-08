function [x_traj,y_traj,alpha_traj,u_alpha,v_alpha] = Controller_super_twisting_continuous(way_R,way_angle,Robot_x,Robot_y,Robot_alpha,theta_ref)
    

    del_T = 0.001;
    N_samples = 1000;
    sum_control = 0;

    K_1 = 0.1;
    K_2 = 0.2;
    %K_3 = 0.01;
    K_3 = 0.0001;

    goal_x = Robot_x + way_R*cos(way_angle);
    goal_y = Robot_y + way_R*sin(way_angle);
    
    x_traj(1) = Robot_x;
    y_traj(1) = Robot_y;
    alpha_traj(1) = Robot_alpha;

    Sliding_surface = zeros(1,N_samples);
    alpha_minus_theta = zeros(1,N_samples);

    R(1) = sqrt( (Robot_x-goal_x)^2 + (Robot_y-goal_y)^2 );
    theta(1) = atan2(Robot_y-goal_y,Robot_x-goal_x);
    alpha(1) = Robot_alpha;

    pos_x(1) = Robot_x - goal_x;
    pos_y(1) = Robot_y - goal_y;

    i = 1;

    %while(R(i)>0.05)
    while(i<2400)

        %time(i) = i*del_T;

    
    alpha_minus_theta(i) = alpha(i) - theta(i);

    if(alpha_minus_theta(i) > pi)
        alpha_minus_theta(i) = -2*pi + alpha_minus_theta(i);
    end

    if(alpha_minus_theta(i) < -pi)
        alpha_minus_theta(i) = 2*pi + alpha_minus_theta(i);
    end

    if(alpha_minus_theta(i)>=(pi/2))
        S = alpha_minus_theta(i)-pi;
    end

    if(alpha_minus_theta(i)<=(-pi/2))
        S = alpha_minus_theta(i)+pi;
    end

    if( ((alpha_minus_theta(i))>(-pi/2)) && ((alpha_minus_theta(i))<(pi/2)) ) 
        S = alpha_minus_theta(i);
    end

    Sliding_surface(i) = S;
    %alpha_minus_theta(i) = alpha(i) - theta(i);
    sum_control = sum_control + K_3*sign(S)*del_T;

    v = -K_1*(R(i))*sign(cos(alpha_minus_theta(i)));
    %v = -3*K_1*sign(cos(alpha_minus_theta(i)));
    omega = -K_2*(abs(S)^0.5)*sign(S) - sum_control + (v/R(i))*sin(alpha_minus_theta(i));
    
        v_alpha(i) = v;
        u_alpha(i) = omega;
        %sum_control = sum_control + K_2*sign(S)*del_T;

        R(i+1) = R(i) + v*cos(alpha(i)-theta(i))*del_T;
        theta(i+1) = theta(i)+(v/R(i))*sin(alpha(i)-theta(i))*del_T;
        alpha(i+1) = alpha(i) + omega*del_T;

        pos_x(i+1) = pos_x(i) + v*cos(alpha(i+1))*del_T;
        pos_y(i+1) = pos_y(i) + v*sin(alpha(i+1))*del_T;
        
        x_traj(i+1) = pos_x(i+1) + goal_x;
        y_traj(i+1) = pos_y(i+1) + goal_y;
        alpha_traj(i+1) = alpha(i+1);

        i = i+1;
    
    end




end