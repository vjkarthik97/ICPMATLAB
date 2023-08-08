function R_opt = R_optimizer(int_goal_x,int_goal_y,Robot_x,Robot_y,T,R_max,angles)

count = 1;

R_op(1) = 0;

R_opt = (int_goal_x-Robot_x)*cos(angles(T)) + (int_goal_y - Robot_y)*sin(angles(T));

if(R_opt > R_max)
    R_opt = R_max;
end
if(R_opt < 0) 
    R_opt = 0;
end
end