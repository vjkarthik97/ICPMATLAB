 function Gap_final = Detect_Gaps(ranges,angles,Robot_x,Robot_y)

c = 1;
Gap_L = 1;
Gap_final = {};
Gap = {};
Fwd_Scan = [];
Bck_Scan = [];
elim_index = [];
thresh = 15;
%Checking for discontinuities
%Forward Scan
for i = 1:1:length(ranges)-1
    if((ranges(i+1) - ranges(i)) > thresh)
        %polarplot(angles(i),ranges(i),'*r','MarkerSize',7);
        %hold on;
        Fwd_Scan(c) = i
        c = c+1;
    end
end

c = 1;
%Backward Scan
for i = length(ranges):-1:2
    if(ranges(i) - ranges(i-1) < -thresh)
        %polarplot(angles(i),ranges(i),'*g','MarkerSize',7);
        %hold on;
        Bck_Scan(c) = i
        c = c+1;
    end
end

%Checking for gaps
if(~isempty(Fwd_Scan)) 
for i = 1:1:length(Fwd_Scan)
    Gap_F_x = Robot_x + ranges(Fwd_Scan(i))*cos(angles(Fwd_Scan(i))); %First side of gap_x
    Gap_F_y = Robot_y + ranges(Fwd_Scan(i))*sin(angles(Fwd_Scan(i))); %First side of gap_y
    
    min_D = inf;
    j = mod(Fwd_Scan(i)+1,length(ranges));
    eff_j = j;
    %while(eff_j ~= mod((Fwd_Scan(i) + (length(ranges)/2)),length(ranges)) )
    for j = Fwd_Scan(i)+1:1:min((Fwd_Scan(i) + (length(ranges)/2)),length(ranges))
        
        eff_j = j;
        %if(j>length(ranges))
            %eff_j = j-length(ranges);
        %end
        can_Gap_S_x = Robot_x + ranges(eff_j)*cos(angles(eff_j)); %Second side of gap_x
        can_Gap_S_y = Robot_y + ranges(eff_j)*sin(angles(eff_j)); %Second side of gap_y
        
        D = Euclidean_distance(Gap_F_x,Gap_F_y,can_Gap_S_x,can_Gap_S_y);
        
        if(D < min_D)
            min_D = D;
            Gap_S_x = can_Gap_S_x;
            Gap_S_y = can_Gap_S_y;
            min_ref = eff_j;
        end
        %j = j+1;
    end
    
    Gap(Gap_L,:) = {[Gap_F_x;Gap_F_y],[Gap_S_x;Gap_S_y],Fwd_Scan(i),min_ref};
    Gap_L = Gap_L + 1;
    %polarplot(angles(min_ref),ranges(min_ref),'*g','MarkerSize',7);
    %hold on;
end
end

if(~isempty(Bck_Scan))
%Check for Gaps - Reverse Direction
for i = 1:1:length(Bck_Scan)
    Gap_S_x = Robot_x + ranges(Bck_Scan(i))*cos(angles(Bck_Scan(i))); %First side of gap_x
    Gap_S_y = Robot_y + ranges(Bck_Scan(i))*sin(angles(Bck_Scan(i))); %First side of gap_y
    
    min_D = inf;
    j = mod(Bck_Scan(i)-1,length(ranges));
    t = mod(Bck_Scan(i) - (length(ranges)/2),length(ranges));
    if(t==0)
        t =1;
    end
    eff_j = j;
    %while(eff_j~=t)
    for j = Bck_Scan(i)-1:-1:max(Bck_Scan(i) - (length(ranges)/2),1)
    %for j = mod(Bck_Scan(i)-1,length(ranges)):-1:mod(Bck_Scan(i) - (length(ranges)/2),length(ranges))
        eff_j = j;
        %if(j<=0)
            %eff_j = length(ranges)+j;
        %end
        can_Gap_F_x = Robot_x + ranges(eff_j)*cos(angles(eff_j)); %Second side of gap_x
        can_Gap_F_y = Robot_y + ranges(eff_j)*sin(angles(eff_j)); %Second side of gap_y
        
        D = Euclidean_distance(can_Gap_F_x,can_Gap_F_y,Gap_S_x,Gap_S_y);
        
        if(D < min_D)
            min_D = D;
            Gap_F_x = can_Gap_F_x;
            Gap_F_y = can_Gap_F_y;
            min_ref = eff_j;
        end
        
        %j = j-1;
    end
    
    Gap(Gap_L,:) = {[Gap_F_x;Gap_F_y],[Gap_S_x;Gap_S_y],min_ref,Bck_Scan(i)};
    Gap_L = Gap_L + 1;

end
end

%To eliminate the gaps within gaps
c = 1;
for i = 1:1:Gap_L-1
    for j = 1:1:Gap_L-1
    
        if(j == i)
            continue;
        else
         if ( (cell2mat(Gap(i,3)) >= cell2mat(Gap(j,3))) && (cell2mat(Gap(i,4)) <= cell2mat(Gap(j,4))) )
             elim_index(c) = i;
             c = c+1;
         end
        end
    
    end
end

if(~isempty(elim_index))
    c = 1;
    for k = 1:1:Gap_L-1

        check = find(elim_index == k);
        TF = isempty(check);
    
        if(TF)
            Gap_final(c,:) = Gap(k,:);
            c = c+1;
        end
    end
else
    Gap_final = Gap;
end

end
