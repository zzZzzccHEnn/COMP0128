%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%  Code modified by Francisco Vasconcelos from
%%%%
%%%%  Drone class, feel free to add functionality as you see fit
%%%%  Author: Daniel Butters
%%%%  Date: 16/11/17
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
classdef Drone_Q1 < handle
    properties (Constant)
        %width, length, height offset between centre and rotors
        body = [1 1 0];
        
        %time interval for simulation (seconds)
        time_interval = 0.02;
        
        % size of floating window that follows drone
        axis_size = 2.;
        
        %colours of each component of drone model
        colours = [[.8 .3 .1];[.2 .2 .5];[.8 .1 .3];[.9 .6 .8];[.9 .2 .4]];

        % mass of drone
        m = 0.2;

        % gravitational acceleration
        g = 9.8;

        % frinction constant
        kd = 0.1;

        % propeller constants
        k = 1;
        b = 0.1;

        % the length from each propeller to the center of the quadcopter
        L = 0.2;

        % quadcopter rotational inertia matrix
        I = [1,0,0;0,1,0;0,0,0.5];
        
        %Follows the drone within the figure
        %Don't use if you are simulating more than one drone!
        %Switch to false to see the overall world view
        drone_follow = true;
    end
    properties
        %axis to draw on
        axis

        %define the acceleration of the drone
        a
        
        %length of one side of the flight arena
        spaceDim
        
        %limits of flight arena
        spaceLimits
        
        %drone position
        x 

        %drone velocity
        xdot

        % input to the motors
        inputs = 0.49 * ones(4,1);
        
        %drone rotation
        theta

        thetadot

        omega

        %drone rotation matrix
        R
        
        %Simulation time
        time
        
        %number of drones
        num_drones
    end
    methods
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %INSTANTIATION OF CLASS
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function obj = Drone_Q1(axis, spaceDim, num_drones)
            if nargin > 1
                obj.axis = axis;

                obj.a = 0;
                
                obj.spaceDim = spaceDim;
                
                obj.spaceLimits = [(-spaceDim/2)+10 (spaceDim/2)-10 (-spaceDim/2)+10 (spaceDim/2)-10 10 spaceDim-10];
                
                % define the initial position
                obj.x = [0;0;5];

                obj.xdot = zeros(3,1);

                obj.theta = zeros(3,1);

                obj.thetadot = zeros(3,1);

                obj.omega = zeros(3,1);
                
                obj.R = [1,0,0;0,1,0;0,0,1];
                
                obj.time = 0;
                
                obj.num_drones = num_drones;
            else
                error('Drone not initialised correctly')
            end
        end
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %DRAWING OF DRONE TO FIGURE
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function draw(obj)
            %how big should the moving window be
            cL = obj.axis_size;
            
            %set to false if you want to see world view
            %if(obj.drone_follow)
            %    axis([obj.pos(1)-cL obj.pos(1)+cL obj.pos(2)-cL obj.pos(2)+cL obj.pos(3)-cL obj.pos(3)+cL]);
            %end
            
            %create middle sphere
            [X Y Z] = sphere(8);
            %[X Y Z] = (obj.body(1)/5.).*[X Y Z];
            X = (obj.body(1)/5.).*X + obj.x(1);
            Y = (obj.body(1)/5.).*Y + obj.x(2);
            Z = (obj.body(1)/5.).*Z + obj.x(3);
            s = surf(obj.axis,X,Y,Z);
            set(s,'edgecolor','none','facecolor',obj.colours(1,:));
            
            %create side spheres
            %front, right, back, left
            hOff = obj.body(3)/2;
            Lx = obj.body(1)/2;
            Ly = obj.body(2)/2;
            rotorsPosBody = [...
                0    Ly    0    -Ly;
                Lx    0    -Lx   0;
                hOff hOff hOff hOff];
            rotorsPosInertial = zeros(3,4);
            for i = 1:4
                rotorPosBody = rotorsPosBody(:,i);
                rotorsPosInertial(:,i) = bodyToInertial(obj,rotorPosBody);
                [X Y Z] = sphere(8);
                X = (obj.body(1)/8.).*X + obj.x(1) + rotorsPosInertial(1,i);
                Y = (obj.body(1)/8.).*Y + obj.x(2) + rotorsPosInertial(2,i);
                Z = (obj.body(1)/8.).*Z + obj.x(3) + rotorsPosInertial(3,i);
                s = surf(obj.axis,X,Y,Z);
                set(s,'edgecolor','none','facecolor',obj.colours(i+1,:));
            end
            obj.axis.Title.String = ['Sim Time = ',num2str(obj.time,'%f'),' seconds'];
        end
        
        function vectorInertial = bodyToInertial(obj, vectorBody)
            vectorInertial = obj.R*vectorBody;
        end
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %SIMULATION FUNCTIONS
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
        %demo (not useful) code to show varying position and rotation
        %replace with your own functions!
        function obj = change_pos_and_orientation(obj)
            t = obj.time;
            dt = obj.time_interval;

            if t<=2
                obj.inputs = obj.inputs;
            elseif t>2 && t<=4
                obj.inputs = 0.49 * ones(4,1) * 1.15;
            elseif t>4
                obj.inputs = 0.49 * [1,1,0,1] * 1.15;
            end

            obj.omega = thetadot2omega(obj);
            obj.a = acceleration(obj);      
            omegadot = angular_acceleration(obj);
            obj.omega = obj.omega + dt * omegadot;
            obj.thetadot = omega2thetadot(obj);
            obj.theta = obj.theta + dt * obj.thetadot;
            obj.xdot = obj.xdot + dt * obj.a;
            obj.x = obj.x + dt * obj.xdot;           
        end

        function omega = thetadot2omega(obj)
            omega = [1,0,-sin(obj.theta(2));
                0,cos(obj.theta(1)),cos(obj.theta(2))*sin(obj.theta(1));
                0,-sin(obj.theta(1)),cos(obj.theta(2))*cos(obj.theta(1))] * obj.thetadot;
        end

        function a = acceleration(obj)
            gravity = [0;0;-obj.g];
            obj.R = rotation(obj);
            T = obj.R * thrust(obj);
            Fd = -obj.kd * obj.xdot;
            a = gravity + 1/obj.m *T + Fd;
        end

        function R = rotation(obj)
            R = [cos(obj.theta(3))*cos(obj.theta(2)), cos(obj.theta(3))*sin(obj.theta(1))*sin(obj.theta(2))-cos(obj.theta(1))*sin(obj.theta(3)), sin(obj.theta(1))*sin(obj.theta(3))+cos(obj.theta(1))*cos(obj.theta(3))*sin(obj.theta(2));
                cos(obj.theta(2))*sin(obj.theta(3)), cos(obj.theta(1))*cos(obj.theta(3))+sin(obj.theta(1))*sin(obj.theta(3))*sin(obj.theta(2)), cos(obj.theta(1))*sin(obj.theta(3))*sin(obj.theta(2))-cos(obj.theta(3))*sin(obj.theta(1));
                -sin(obj.theta(2)), cos(obj.theta(2))*sin(obj.theta(1)), cos(obj.theta(1))*cos(obj.theta(2))];
        end
        
        function T = thrust(obj)
            T = [0;0;obj.k*sum(obj.inputs, 'all')];
        end

        function tau = torques(obj)
            tau = [obj.L * obj.k * (obj.inputs(1) - obj.inputs(3));
                obj.L * obj.k * (obj.inputs(2) - obj.inputs(4));
                obj.b * (obj.inputs(1) - obj.inputs(2) + obj.inputs(3) - obj.inputs(4))];
        end    

        function thetadot = omega2thetadot(obj)
            thetadot = inv([1,0,-sin(obj.theta(2));
                0,cos(obj.theta(1)),cos(obj.theta(2))*sin(obj.theta(1));
                0,-sin(obj.theta(1)),cos(obj.theta(2))*cos(obj.theta(1))]) * obj.omega;
        end
                
        function omegadot = angular_acceleration(obj)
            tau = torques(obj);
            omegadot = inv(obj.I) * (tau - cross(obj.omega, obj.I * obj.omega)); 

        end
        
        function update(obj)
            %update simulation time
            obj.time = obj.time + obj.time_interval;
            
            %change position and orientation of drone
            obj = change_pos_and_orientation(obj);
            
            %draw drone on figure
            draw(obj);
        end
    end
end

