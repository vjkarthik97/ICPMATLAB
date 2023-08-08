function [R,I] = Sensor_data_process(angles,ranges,T)
     
    del_theta = 2*pi/length(ranges);
    
    norm_R = inf*ones(1,length(ranges));
    
for i = T-((length(ranges)-2)/4):1:T+((length(ranges)-2)/4)

    if(i<=0)
        eff_i = i + length(ranges);
    elseif(i>length(ranges))
        eff_i = i - length(ranges);
    else
        eff_i = i;
    end
    norm_R(eff_i) = ranges(eff_i)/(2*cos((T-i)*del_theta));
end

    [R,I] = min(norm_R);
end