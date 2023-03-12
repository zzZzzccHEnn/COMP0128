%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%  Code modified by Francisco Vasconcelos from
%%%%
%%%%  Visualisation code for quadcopter 
%%%%  Author: Daniel Butters
%%%%  Date: 16/11/17
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
time_Q2 = [];

num_drones = 1;

%instantiate a drone object, input the axis and arena limits
drones = [];
for i = 1:num_drones
    drones = [drones Drone_Q2(ax1, spaceDim, num_drones)];
end

while(drones(1).time < 10.0)
    %clear axis
    cla(ax1);
    
    %update and draw drones
    for i = 1:num_drones
        update(drones(i));
    end

    Drone_trajectory = [Drone_trajectory drones.state];
    x_Q2 = Drone_trajectory(1, 1:width(Drone_trajectory));
    y_Q2 = Drone_trajectory(2, 1:width(Drone_trajectory));
    z_Q2 = Drone_trajectory(3, 1:width(Drone_trajectory));
    roll_Q2 = Drone_trajectory(7, 1:width(Drone_trajectory));
    pitch_Q2 = Drone_trajectory(8, 1:width(Drone_trajectory));
    yaw_Q2 = Drone_trajectory(9, 1:width(Drone_trajectory));

    time_Q2 = [time_Q2 drones.time];

    plot3(ax1,x_Q2, y_Q2, z_Q2);
    plot3(ax1, x_Q2, y_Q2, zeros(1, length(z_Q2)), 'r');
       
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

save('Q2_data');

%%
non_linear = load('Q1_data.mat');

subplot(3,2,1)
title('position along X axis')
yyaxis left
plot(time_Q2, non_linear.x_Q1);
ylabel('non-linear');
yyaxis right
plot(time_Q2, x_Q2);
ylabel('linear');

subplot(3,2,2)
title('position along Y axis')
yyaxis left
plot(time_Q2, non_linear.y_Q1);
ylabel('non-linear');
yyaxis right
plot(time_Q2, y_Q2);
ylabel('linear');

subplot(3,2,3)
title('position along Z axis')
yyaxis left
plot(time_Q2, non_linear.z_Q1);
ylabel('non-linear');
yyaxis right
plot(time_Q2, z_Q2);
ylabel('linear');

subplot(3,2,4)
title('Roll')
yyaxis left
plot(time_Q2, non_linear.roll_Q1);
ylabel('non-linear');
yyaxis right
plot(time_Q2, roll_Q2);
ylabel('linear');

subplot(3,2,5)
title('Pitch')
yyaxis left
plot(time_Q2, non_linear.pitch_Q1);
ylabel('non-linear');
yyaxis right
plot(time_Q2, pitch_Q2);
ylabel('linear');

subplot(3,2,6)
title('Yaw')
yyaxis left
plot(time_Q2, non_linear.yaw_Q1);
ylabel('non-linear');
yyaxis right
plot(time_Q2, yaw_Q2);
ylabel('linear');


