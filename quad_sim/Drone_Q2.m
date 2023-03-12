classdef Drone_Q2 < handle
    properties(Constant)
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

        % the state of the system
        state

        % the dynamics of the system
        dynamics
        
        %drone position
        x 

        %drone velocity
        xdot

        % input to the motors
        inputs

        % input at the equlibrium point
        inputs_0
        
        % pitch, roll and yaw of the drone rotation
        theta
        
        % devirative of pitch, roll and yaw
        thetadot
        
        % angular velocity vector
        omega

        % devirative of angular velocity vector
        omegadot

        %drone rotation matrix
        R
        
        %Simulation time
        time
        
        %number of drones
        num_drones

        A

        B
    end

    methods
         %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %INSTANTIATION OF CLASS
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function obj = Drone_Q2(axis, spaceDim, num_drones)
            if nargin > 1
                jacobian = load('jacobian.mat');
                obj.A = jacobian.disc_sys.A;
                obj.B = jacobian.disc_sys.B;

                obj.inputs_0 = [obj.m*obj.g/(4*obj.k);obj.m*obj.g/(4*obj.k);obj.m*obj.g/(4*obj.k);obj.m*obj.g/(4*obj.k)];

                obj.inputs = zeros(4,1);

                obj.axis = axis;

                obj.a = zeros(3,1);
                
                obj.spaceDim = spaceDim;
                
                obj.spaceLimits = [(-spaceDim/2)+10 (spaceDim/2)-10 (-spaceDim/2)+10 (spaceDim/2)-10 10 spaceDim-10];
                
                % define the initial position
                obj.x = [0;0;5];

                obj.xdot = zeros(3,1);

                obj.theta = zeros(3,1);

                obj.thetadot = zeros(3,1);

                obj.omega = zeros(3,1);

                obj.omegadot = zeros(3,1);
                
                obj.R = [1,0,0;0,1,0;0,0,1];

                obj.time = 0;

                obj.state = [obj.x;obj.xdot;obj.theta;obj.omega];

                obj.dynamics = [obj.xdot;obj.a;obj.thetadot;obj.omegadot];
                
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
        function obj = change_pos_and_orientation(obj)
            t = obj.time;
            dt = obj.time_interval;
    
            if t<=2
                obj.inputs = 0.49 * [1;1;1;1];
            elseif t>2 && t<=4
                obj.inputs = 1.15 * 0.49 * [1;1;1;1];
            elseif t>4
                obj.inputs = 0.49 * [1;1;0;1] * 1.15;
            end

            obj.state = obj.A*obj.state + obj.B*(obj.inputs - obj.inputs_0);

            obj.x = obj.state(1:3);
            obj.xdot = obj.state(4:6);
            obj.theta = obj.state(7:9);
            obj.thetadot = obj.state(10:12);

            obj.R = rotation(obj);
            
        end

        function R = rotation(obj)
            R = [cos(obj.theta(3))*cos(obj.theta(2)), cos(obj.theta(3))*sin(obj.theta(1))*sin(obj.theta(2))-cos(obj.theta(1))*sin(obj.theta(3)), sin(obj.theta(1))*sin(obj.theta(3))+cos(obj.theta(1))*cos(obj.theta(3))*sin(obj.theta(2));
                cos(obj.theta(2))*sin(obj.theta(3)), cos(obj.theta(1))*cos(obj.theta(3))+sin(obj.theta(1))*sin(obj.theta(3))*sin(obj.theta(2)), cos(obj.theta(1))*sin(obj.theta(3))*sin(obj.theta(2))-cos(obj.theta(3))*sin(obj.theta(1));
                -sin(obj.theta(2)), cos(obj.theta(2))*sin(obj.theta(1)), cos(obj.theta(1))*cos(obj.theta(2))];
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




