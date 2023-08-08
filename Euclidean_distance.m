function D = Euclidean_distance(Gap_F_x,Gap_F_y,can_Gap_S_x,can_Gap_S_y)

    D = ((Gap_F_x - can_Gap_S_x)^2 + (Gap_F_y - can_Gap_S_y)^2)^(0.5); 
end