function [D_best,i_best] = constrained_angle_search(D_check,alpha_robot)

alpha_robot = mod(alpha_robot+pi,2*pi)-pi;
(alpha_robot)*360/(2*pi)
D_best = inf;

delta = 20;

alpha_robot_N = round(1 + (257/(2*pi))*(alpha_robot+pi));

min_index = alpha_robot_N - delta;
max_index = alpha_robot_N + delta;

if(min_index<0)
    min_index = 258+min_index;
end

if(max_index>258)
    max_index = max_index-258;
end

count = min_index;

while(mod(count,258)+1~=max_index)
    if(D_check(mod(count,258)+1)<D_best)
        D_best = D_check(mod(count,258)+1);
        i_best = mod(count,258)+1;
    end
    count = count+1;
end

if(alpha_robot<0)
    alpha_robot = alpha_robot+pi;
else
    alpha_robot = alpha_robot-pi;
end
%alpha_robot = mod(alpha_robot+pi,2*pi)-pi;
alpha_robot_N = round(1 + (257/(2*pi))*(alpha_robot+pi));

min_index = alpha_robot_N - delta;
max_index = alpha_robot_N + delta;

if(min_index<0)
    min_index = 258+min_index;
end

if(max_index>258)
    max_index = max_index-258;
end

count = min_index;

while(mod(count,258)+1~=max_index)
    if(D_check(mod(count,258)+1)<D_best)
        D_best = D_check(mod(count,258)+1);
        i_best = mod(count,258)+1;
    end
    count = count+1;
end

end
