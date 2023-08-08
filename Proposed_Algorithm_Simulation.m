clc;
close all;
clear all;

load exampleMaps.mat
set(gca,'fontsize', 22)
Gap_global = [];
int_goal_global = [];
x = [];
y = [];
alpha = [];

alpha(1) = 0;
Start_x = 35;
Start_y = 10;
alpha_traj = [];

int_goal_x = Start_x;
int_goal_y = Start_y;

%Final_Goal_x = 10;
%Final_Goal_y = 90;

Final_Goal_x = 10;
Final_Goal_y = 90;

u = [];
v = [];

c = 1;
way_count = 0;
image = imread('Custom_Map_Maze_criss_cross.png');
grayimage = rgb2gray(image);
bwimage = grayimage < 0.5;
refMap = binaryOccupancyMap(bwimage);
%show(grid)

%refMap = binaryOccupancyMap(simpleMap,1);
refFigure = figure('Name','SimpleMap');
show(refMap);
hold on;
plot(Start_x,Start_y,'^r','MarkerFaceColor','#FF0000','MarkerSize',10)
hold on;
plot(Final_Goal_x,Final_Goal_y,'sg','MarkerFaceColor','#00FF00','MarkerSize',10)
hold on;

%[mapdimx,mapdimy] = size(simpleMap);
mapdimx = 100;
mapdimy = 100;

map = binaryOccupancyMap(mapdimy,mapdimx,10);
mapFigure = figure('Name','Unknown Map');
%show(map);

%diffDrive = differentialDriveKinematics("VehicleInputs","VehicleSpeedHeadingRate");
%controller = controllerPurePursuit('DesiredLinearVelocity',2,'MaxAngularVelocity',3);

lidar = rangeSensor;
lidar.Range = [0,100];

position = [Start_x Start_y 0];
Robot_x = position(1);
Robot_y = position(2);

figure(mapFigure)
[ranges, angles] = lidar(position,refMap);
scan = lidarScan(ranges,angles);
validScan = removeInvalidData(scan,'RangeLimits',[0,lidar.Range(2)]);
insertRay(map,position,validScan,lidar.Range(2));
show(map);
hold on;
plot(Robot_x,Robot_y,'xr','MarkerSize',10) % Grid location center
%path = [20 20; 20-4*cos(pi/4) 20-4*sin(pi/4);];

%figure(3)
%polarplot(angles,ranges);
%hold on;
k = 1;

while(Euclidean_distance(Robot_x,Robot_y,Final_Goal_x,Final_Goal_y)>2)
    
    [ranges, angles] = lidar(position,refMap);
    
    ranges_gap = ranges;
    angles_gap = angles;

    Robot_x_gap = Robot_x;
    Robot_y_gap = Robot_y;
    
    Gap = [];
    
    Gap = Detect_Gaps(ranges,angles,Robot_x,Robot_y); %Detect Gaps in this step

    max_ct = length(Gap(:,1));
    ct = 1;

    angle_goal_gap = [];

    Gap_save = [];
    int_goal_save = [];
    z = 1;
%Identifying closest gap
    while(ct <= max_ct)
        
        F_end = cell2mat(Gap(ct,1));
        S_end = cell2mat(Gap(ct,2));
        
        F_side_i = cell2mat(Gap(ct,3));
        S_side_i = cell2mat(Gap(ct,4));
    
        %Gap_F_x = Robot_x + ranges(F_side_i)*cos(angles(F_side_i));
        %Gap_F_y = Robot_y + ranges(F_side_i)*sin(angles(F_side_i));

        %Gap_S_x = Robot_x + ranges(S_side_i)*cos(angles(S_side_i));
        %Gap_S_y = Robot_y + ranges(S_side_i)*sin(angles(S_side_i));
        
        Gap_F_x = F_end(1);
        Gap_F_y = F_end(2);

        Gap_S_x = S_end(1);
        Gap_S_y = S_end(2);
        
        if(Gap_F_x == 0 && Gap_F_y == 0 && Gap_S_x == 0 && Gap_S_y == 0)
            ct = ct+1;
            continue; 
        end
        %flag_check = 0;
        %if(~isempty(Gap_global))
            %flag_check = Gap_search(Gap_global,0.5*(Gap_F_x+Gap_S_x),0.5*(Gap_F_y+Gap_S_y));
            %if(flag_check == 1)
                %ct = ct+1;
                %continue;
            %end
        %end
      
        angle_robot_goal = (atan2(Final_Goal_y - Robot_y , Final_Goal_x - Robot_x));
        
        %C_1 = min(abs(angles(F_side_i) - angle_robot_goal),mod(abs(angles(F_side_i) - angle_robot_goal),pi));
        
        %C_2 = min(abs(angles(S_side_i) - angle_robot_goal),mod(abs(angles(S_side_i) - angle_robot_goal),pi));
        
        
        %C_1 = abs(atan2(Gap_F_y - Final_Goal_y,Gap_F_x - Final_Goal_x));
         
        %C_2 = abs(atan2(Gap_S_y - Final_Goal_y,Gap_S_x - Final_Goal_x));
        
        if(abs(angle_robot_goal-angles(F_side_i))>pi)
            C_1 = 2*pi - abs(angle_robot_goal-angles(F_side_i));
        else
            C_1 = abs(angle_robot_goal-angles(F_side_i));
        end
        
        if(abs(angle_robot_goal-angles(S_side_i))>pi)
            C_2 = 2*pi - abs(angle_robot_goal-angles(S_side_i));
        else
            C_2 = abs(angle_robot_goal-angles(S_side_i));
        end
        %C_1 = mod(abs(angle_robot_goal-angles(F_side_i)),pi);
        %C_2 = mod(abs(angle_robot_goal-angles(S_side_i)),pi);
        angle_goal_gap(z) = min(C_1,C_2); 
        
        %F_end = cell2mat(Gap(ct,1));
        %S_end = cell2mat(Gap(ct,2));
        Gap_save(z,:) = [Gap_F_x Gap_F_y Gap_S_x Gap_S_y];
        int_goal_save(z,:) = [0.5*(Gap_F_x+Gap_S_x) 0.5*(Gap_F_y+Gap_S_y)];
        
        Gap_global = [Gap_global ; Gap_save(z,:)];
        int_goal_global = [int_goal_global;int_goal_save(z,:)];
        
        %angle_goal_gap_global = [angle_goal_gap_global ; angle_goal_gap(z)]
        ct = ct+1;
        z = z+1;
    end
    
    %Gap_global = [Gap_global ; Gap_save];
    %int_goal_global = [int_goal_global;int_goal_save];
    
    %Plotting the Navigation Tree
    nt_count = 1;
    while(nt_count<= length(Gap_save(:,1)))
        
        mid_x = 0.5*(Gap_save(nt_count,1)+Gap_save(nt_count,3));
        mid_y = 0.5*(Gap_save(nt_count,2)+Gap_save(nt_count,4));
        if((mid_x == 0) && (mid_y==0))
            nt_count = nt_count+1;
            continue;
        end
        %figure(mapFigure)
        %line([int_goal_x mid_x],[int_goal_y mid_y],'LineWidth',1); 
        %hold on;
        nt_count = nt_count+1;
    
    end
    
    z = 1;
    %figure(mapFigure)

    [min_angle_goal_gap,min_angle_goal_i] = min(angle_goal_gap);
    
    if(~isempty(Gap_save))
        
        while(z<=length(Gap_save(:,1)))
            %figure(mapFigure)
            if((0.5*(Gap_save(z,1)+Gap_save(z,3))==0) && (0.5*(Gap_save(z,2)+Gap_save(z,4))==0))
                z=z+1;
                continue;
            end
            %plot(0.5*(Gap_save(z,1)+Gap_save(z,3)),0.5*(Gap_save(z,2)+Gap_save(z,4)),'o','MarkerSize',14);
            hold on;
            %figure(refFigure)
            %line([Gap_save(z,1) Gap_save(z,3)],[Gap_save(z,2) Gap_save(z,4)],'LineWidth',2,'LineStyle','-.');
            %hold on;
            z = z+1;
        end
    
        
        %int_goal = (cell2mat(Gap(min_angle_goal_i,1)) + cell2mat(Gap(min_angle_goal_i,2)))/2;
        int_goal = int_goal_save(min_angle_goal_i,:);
        int_goal_x = int_goal(1);
        int_goal_y = int_goal(2);
    
    end

    angle_goal_robot = atan2(Final_Goal_y - Robot_y , Final_Goal_x - Robot_x);

    range_index = round((length(ranges)/2) + (angle_goal_robot/(2*pi))*length(ranges));

    if(ranges(range_index) > Euclidean_distance(Robot_x,Robot_y,Final_Goal_x,Final_Goal_y)) %Checking if there exists an obtacle in the direction to goal
        int_goal_x = Final_Goal_x; 
        int_goal_y = Final_Goal_y;
    else
        int_goal_x = int_goal(1);
        int_goal_y = int_goal(2);
    end
 
%Navigating to the intermediate goal
    while(Euclidean_distance(Robot_x,Robot_y,int_goal_x,int_goal_y) > 0.5)  
    
        [ranges, angles] = lidar(position,refMap);
        
        %if(isempty(alpha_traj))
            %Robot_alpha = (-pi/2);
        %else
            Robot_alpha = mod(alpha(end)+pi,2*pi)-pi;
        %end
        
        alpha_N = round(1 + (257/(2*pi))*(Robot_alpha+pi));
        
        R_opt = [];
        D_check = [];
        R_max = [];
        for T = 1:1:length(ranges)
        %for T = alpha_N-25:1:alpha_N+25
    
            [R_max(T),I] = Sensor_data_process(angles,ranges,T); %Obtaining the maximum possible radius of the circle in a given direction
    
            R_opt(T) = R_optimizer(int_goal_x,int_goal_y,Robot_x,Robot_y,T,R_max(T),angles); %Optimal R in terms closest distance to intermediate goal
            D_check(T) = Euclidean_distance(int_goal_x,int_goal_y, Robot_x + R_opt(T)*cos(angles(T)),Robot_y + R_opt(T)*sin(angles(T)));
        end
        
        begin_index = mod(alpha_N-10,258);
        end_index = mod(alpha_N+10,258);
        %[D_best,i_best] = min(D_check);
        [D_best,i_best] = constrained_angle_search(D_check,alpha(end));
        %i_best
        x_traj = [];
        y_traj = [];
        
        %alpha(end)

        %Determining the angle at which the robot should be stabilized at the next
        %waypoint
        way_x = Robot_x + R_opt(i_best)*cos(angles(i_best));
        way_y = Robot_y + R_opt(i_best)*sin(angles(i_best));
        way_count = way_count+1;
        fprintf('Current Waypoint count:%f',way_count)

        %Gap = Detect_Gaps(ranges,angles,Robot_x,Robot_y); %Detect Gaps in this step

        %if(Euclidean_distance(way_x,way_y,int_goal_x,int_goal_y) < 0.2)
            %theta_ref = 0*ref_angle_int_goal(ranges_gap,angles_gap,int_goal_x,int_goal_y,Gap,Robot_x,Robot_y);
        %else
            %theta_ref =  atan2(way_y-int_goal_y,way_x-int_goal_x);
        %end
        
        %theta_ref = mod(theta_ref+pi,2*pi) - pi;
        %theta_ref = sign(alpha(end))*5*pi;
        %theta_ref = mod(theta_ref+pi,2*pi) - pi;
        %theta_ref = ref_angle_int_goal(ranges_gap,angles_gap,int_goal_x,int_goal_y,Gap,Robot_x_gap,Robot_y_gap);
        %[x_traj,y_traj,alpha_traj,u_alpha,v_alpha] = Controller_novel(R_opt(i_best),angles(i_best),Robot_x,Robot_y,Robot_alpha,theta_ref);
        [x_traj,y_traj,alpha_traj,u_alpha,v_alpha] = Controller_super_twisting_continuous(R_opt(i_best),angles(i_best),Robot_x,Robot_y,Robot_alpha,0);
        alpha_traj = mod(alpha_traj+pi,2*pi)-pi;
        x = [x x_traj];
        y = [y y_traj];
        alpha = [alpha alpha_traj];
        u = [u u_alpha];
        v = [v v_alpha];

        %Robot_x = Robot_x + R_opt(i_best)*cos(angles(i_best));
        %Robot_y = Robot_y + R_opt(i_best)*sin(angles(i_best));

        Robot_x = x(end);
        Robot_y = y(end);

        %Robot_x = x_traj(end);
        %Robot_y = y_traj(end);

        %position = [Robot_x Robot_y 0];

        %figure(mapFigure)
        %scatter(Robot_x,Robot_y)
        %hold on;
        %viscircles([Robot_x,Robot_y],R_opt(i_best),'LineStyle','--','Color','g');
        %hold on;
        %[ranges, angles] = lidar(position,refMap);
        %scan = lidarScan(ranges,angles);
        %validScan = removeInvalidData(scan,'RangeLimits',[0,lidar.Range(2)]);
        %insertRay(map,position,validScan,lidar.Range(2));
        %show(map);
        %hold on;
        %plot(int_goal_x,int_goal_y,'ok','MarkerSize',14)
        %hold on;
        %plot(Robot_x,Robot_y,'xb','MarkerSize',14)
        %hold on;
        set(gca,'fontsize', 22)

        figure(refFigure)
        plot(int_goal_x,int_goal_y,'ok','MarkerSize',14)
        hold on;
        %viscircles([way_x,way_y],R_opt(i_best),'LineStyle','-','Color','g');
        %hold on;
        %plot(Robot_x,Robot_y,'ob','MarkerSize',4)
        %hold on;
        plot(way_x,way_y,'ob','MarkerSize',4)
        hold on;
        plot(x_traj,y_traj,'or','MarkerSize',4);
        hold on;


        position = [Robot_x Robot_y 0];
        %figure(mapFigure)
        %scatter(Robot_x,Robot_y)
        %hold on;
        grid on

    end
    %figure(mapFigure)
    %scatter(Robot_x,Robot_y,'o')
    hold on;
    k = k+1;
    
    %figure(4)
    %[ranges, angles] = lidar(position,refMap);
    %polarplot(angles,ranges);
end

%{
figure(refFigure)
x_axis_pos = linspace(-3.5,3.5,100);
y_angle_pos = x_axis_pos*tan(alpha(1));
s_pos = plot(x_axis_pos,y_angle_pos,'LineWidth',4,'LineStyle','--');
hold on;
scatter(x(1),y(1),'filled','b')
hold on;
r = plot(x(1),y(1),'LineWidth',2);
hold on;
p = plot(x(1),y(1),'o','MarkerFaceColor','red');
hold off;
xlim([0 100])
ylim([0 100])
ax = gca;
ax.XAxisLocation = 'origin';
ax.YAxisLocation = 'origin';
ax.XLabel.String = 'X';
ax.YLabel.String = 'Y';
ax.Title.String = 'Robot Motion';
grid on;

for k = 1:7500:length(x)
    
y_angle_pos = tan(alpha(k))*x_axis_pos;
s_pos.XData = x_axis_pos + x(k);
s_pos.YData = y_angle_pos +y(k);
hold on;
p.XData = x(k);
p.YData = y(k);
hold on;
r.XData = x(1:k);
theta_ref = mod(theta_ref+pi,2*pi) - pi;r.YData = y(1:k);
drawnow
end
%}
%{
figure(3)
plot(u(1:1:end),'LineWidth',2);
hold on;
plot(v(1:1:end),'LineWidth',2);
grid on;
legend('u_{\alpha}','v');
xlabel('Time(x0.001) seconds');
%scatter(int_goal_global(:,1),int_goal_global(:,2))
%%}
%}