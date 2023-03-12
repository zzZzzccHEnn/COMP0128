%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%  Code modified by Francisco Vasconcelos from
%%%%
%%%%  Visualisation code for quadcopter 
%%%%  Author: Daniel Butters
%%%%  Date: 16/11/17
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%% coding for Q3_a %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clear all;
close all;

%Define total width, length and height of flight arena (metres)
spaceDim = 10;
spaceLimits = [-spaceDim spaceDim -spaceDim spaceDim 0 spaceDim+1];

%do you want to draw a ground image on the figure?
draw_ground = false;
if(draw_ground)
    ground_img = imread('ground.png');
end


%figure to display drone simulation
f1 = figure;
ax1 = gca;
view(ax1, 3);
axis equal;
axis(spaceLimits)
grid ON
grid MINOR
caxis(ax1, [0 spaceDim]);
hold(ax1,'on')
axis vis3d

Drone_trajectory = [];
time = [];

num_drones = 1;

%instantiate a drone object, input the axis and arena limits
drones = [];
for i = 1:num_drones
    drones = [drones Drone_Q3a(ax1, spaceDim, num_drones)];
end

t = 0:0.02:100.0;

for time = 1:(length(t)-1)
% while(drones(1).time < 15.0)
    %clear axis
    cla(ax1);
    
    %update and draw drones
    for i = 1:num_drones
        update(drones(i));
    end

    Drone_trajectory = [Drone_trajectory [drones.x; drones.theta]];
    x_Q3 = Drone_trajectory(1, 1:width(Drone_trajectory));
    y_Q3 = Drone_trajectory(2, 1:width(Drone_trajectory));
    z_Q3 = Drone_trajectory(3, 1:width(Drone_trajectory));
    roll_Q3 = Drone_trajectory(4, 1:width(Drone_trajectory));
    pitch_Q3 = Drone_trajectory(5, 1:width(Drone_trajectory));
    yaw_Q3 = Drone_trajectory(6, 1:width(Drone_trajectory));
    plot3(ax1,x_Q3, y_Q3, z_Q3);
    plot3(ax1, x_Q3, y_Q3, zeros(1, length(z_Q3)), 'r');

    time = [time drones.time];
                
    %optionally draw the ground image
    if(draw_ground)
        imagesc([-spaceDim,spaceDim],[-spaceDim,spaceDim],ground_img);
    end
    
    %apply fancy lighting (optional)
    camlight
    
    %update figure
    drawnow
    pause(0.01)
end

% save('Q3a_data');


%% 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%% coding for ploting %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% disp('max y =');
% display(max(y));
% disp('min y =');
% display(min(y(1000:width(y))));

subplot(3,2,1)
plot(t(1:width(t)-1), x_Q3);
hold on
plot(t(651),x_Q3(651),'r*');
plot(t(1770),x_Q3(1770),'r*');
plot(t(2555),x_Q3(2555),'r*');
plot(t(3291),x_Q3(3291),'r*');
plot(t(4135),x_Q3(4135),'r*');
title('position on X axis');

subplot(3,2,2)
plot(t(1:width(t)-1), y_Q3);
hold on
plot(t(651),y_Q3(651),'r*');
plot(t(1770),y_Q3(1770),'r*');
plot(t(2555),y_Q3(2555),'r*');
plot(t(3291),y_Q3(3291),'r*');
plot(t(4135),y_Q3(4135),'r*');
title('position on Y axis');

subplot(3,2,3)
plot(t(1:width(t)-1), z_Q3);
hold on
plot(t(651),z_Q3(651),'r*');
plot(t(1770),z_Q3(1770),'r*');
plot(t(2555),z_Q3(2555),'r*');
plot(t(3291),z_Q3(3291),'r*');
plot(t(4135),z_Q3(4135),'r*');
title('position on Z axis');

subplot(3,2,4)
plot(t(1:width(t)-1), roll_Q3);
title('Roll');

subplot(3,2,5)
plot(t(1:width(t)-1), pitch_Q3);
title('Pitch');

subplot(3,2,6)
plot(t(1:width(t)-1), yaw_Q3);
title('Yaw');


%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%% coding for Q3_b %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clear all;
close all;

%Define total width, length and height of flight arena (metres)
spaceDim = 10;
spaceLimits = [-spaceDim spaceDim -spaceDim spaceDim 0 spaceDim+1];

%do you want to draw a ground image on the figure?
draw_ground = false;
if(draw_ground)
    ground_img = imread('ground.png');
end


%figure to display drone simulation
f1 = figure;
ax1 = gca;
view(ax1, 3);
axis equal;
axis(spaceLimits)
grid ON
grid MINOR
caxis(ax1, [0 spaceDim]);
hold(ax1,'on')
axis vis3d

Drone_trajectory = [];
time = [];

num_drones = 1;

%instantiate a drone object, input the axis and arena limits
drones = [];
for i = 1:num_drones
    drones = [drones Drone_Q3b(ax1, spaceDim, num_drones)];
end

t = 0:0.02:100.0;

for time = 1:(length(t)-1)
% while(drones(1).time < 15.0)
    %clear axis
    cla(ax1);
    
    %update and draw drones
    for i = 1:num_drones
        update(drones(i));
    end

    Drone_trajectory = [Drone_trajectory [drones.x; drones.theta]];
    x_Q3 = Drone_trajectory(1, 1:width(Drone_trajectory));
    y_Q3 = Drone_trajectory(2, 1:width(Drone_trajectory));
    z_Q3 = Drone_trajectory(3, 1:width(Drone_trajectory));
    roll_Q3 = Drone_trajectory(4, 1:width(Drone_trajectory));
    pitch_Q3 = Drone_trajectory(5, 1:width(Drone_trajectory));
    yaw_Q3 = Drone_trajectory(6, 1:width(Drone_trajectory));
    plot3(ax1,x_Q3, y_Q3, z_Q3);
    plot3(ax1, x_Q3, y_Q3, zeros(1, length(z_Q3)), 'r');

    time = [time drones(1).time];
                
    %optionally draw the ground image
    if(draw_ground)
        imagesc([-spaceDim,spaceDim],[-spaceDim,spaceDim],ground_img);
    end
    
    %apply fancy lighting (optional)
    camlight
    
    %update figure
    drawnow
    pause(0.01)
end

save('Q3b_data');
