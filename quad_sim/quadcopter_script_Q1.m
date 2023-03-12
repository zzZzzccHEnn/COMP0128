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

Drone_position = [];
Drone_orientation = [];
time_Q1 = [];

num_drones = 1;

%instantiate a drone object, input the axis and arena limits
drones = [];
for i = 1:num_drones
    drones = [drones Drone_Q1(ax1, spaceDim, num_drones)];
end

while(drones(1).time < 10.0)
    %clear axis
    cla(ax1);
    
    %update and draw drones
    for i = 1:num_drones
        update(drones(i));
    end

    Drone_position = [Drone_position drones.x];
    x_Q1 = Drone_position(1, 1:width(Drone_position));
    y_Q1 = Drone_position(2, 1:width(Drone_position));
    z_Q1 = Drone_position(3, 1:width(Drone_position));

    Drone_orientation = [Drone_orientation drones.theta];
    roll_Q1 = Drone_orientation(1, 1:width(Drone_orientation));
    pitch_Q1 = Drone_orientation(2, 1:width(Drone_orientation));
    yaw_Q1 = Drone_orientation(3, 1:width(Drone_orientation));

    time_Q1 = [time_Q1 drones.time];

    plot3(ax1,x_Q1, y_Q1, z_Q1);
    plot3(ax1, x_Q1, y_Q1, zeros(1, length(z_Q1)), 'r');
       
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

save('Q1_data');

%%
subplot(2,2,1)
yyaxis left
plot(time_Q1, x_Q1);
ylabel('position on X axis');
yyaxis right
plot(time_Q1, y_Q1);
ylabel('position on Y axis');

subplot(2,2,2)
plot(time_Q1, z_Q1);
ylabel('position on Z axis');

subplot(2,2,3)
yyaxis left
plot(time_Q1, roll_Q1);
ylabel('Roll');
yyaxis right
plot(time_Q1, pitch_Q1);
ylabel('Pitch');

subplot(2,2,4)
plot(time_Q1, yaw_Q1);
ylabel('Yaw');
